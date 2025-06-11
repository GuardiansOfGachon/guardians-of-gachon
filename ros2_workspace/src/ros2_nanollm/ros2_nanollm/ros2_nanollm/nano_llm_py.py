# SPDX-FileCopyrightText: Copyright (c) <year> NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from PIL import Image as im
from nano_llm import NanoLLM, ChatHistory
import numpy as np
import cv2
import time
 

class Nano_LLM_Subscriber(Node):

    def __init__(self):
        super().__init__('nano_llm_subscriber')
        
        #EDIT MODEL HERE 
        self.declare_parameter('model', "Efficient-Large-Model/VILA-2.7b") #inserting vila
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")
      
        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)
        self.query_subscription  # prevent unused variable warning

        # Subscriber for input image
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.image_listener_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge() 
      
        #load the model 
        self.model = NanoLLM.from_pretrained("Efficient-Large-Model/VILA-2.7b")

        #chatHistory var 
        self.chat_history = ChatHistory(self.model)

        ##  PUBLISHER
        self.output_publisher = self.create_publisher(String, 'output', 10)
        self.image_publisher = self.create_publisher(CompressedImage, '/image_with_timestamp', 10)
        
        self.query = (
    "You are an AI security assistant monitoring a road environment. "
    "Analyze the given image and determine which of the following categories applies: "
    "Traffic Accident (vehicle collisions, pedestrian accidents), "
    "Construction Zone (truck, roadwork activity, machinery, barricades), "
    "No Hazard (scene is safe, no threats). "
    "Provide the response in the following format: 'Category / Description'.\n\n"

    "Now, analyze the following image and provide a similar response."
)


    def query_listener_callback(self, msg):
        #can change with user needs 
        self.query = msg.data


    def image_listener_callback(self, data): 
        input_query = self.query
        
        timestamp = int(time.time())        
       
        # call model with input_query and input_image 
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        PIL_img = im.fromarray(cv_img)

        # Parsing input text prompt
        prompt = input_query.strip("][()")
        text = prompt.split(',')
        self.get_logger().info('Your query: %s' % text) #can check to see what the query is 


        #chathistory 
        self.chat_history.append('user', image=PIL_img)
        self.chat_history.append('user', prompt, use_cache=True)
        embedding, _ = self.chat_history.embed_chat()
      
        output = self.model.generate(
            inputs=embedding,
            kv_cache=self.chat_history.kv_cache,
            min_new_tokens = 10,
            max_new_tokens = 128,
            streaming = False,
            do_sample = False,
        )

        #FIX PUBLISHER
        data.header = Header()
        data.header.stamp.sec = timestamp
        self.image_publisher.publish(data)
         
        output_msg = String()
        output_msg.data = f"{timestamp}|{output}"
        self.output_publisher.publish(output_msg)
        self.get_logger().info(f"Published output: {output}")

        self.chat_history.reset()



def main(args=None):
    rclpy.init(args=args)

    nano_llm_subscriber = Nano_LLM_Subscriber()

    rclpy.spin(nano_llm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nano_llm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

