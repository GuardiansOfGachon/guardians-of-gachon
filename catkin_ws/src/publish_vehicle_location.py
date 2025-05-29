#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from morai_msgs.msg import GPSMessage
import json

class GPSToLocationPublisher:
    def __init__(self):
        rospy.init_node('gps_to_location_publisher', anonymous=True)

        # Publisher 초기화
        self.vehicle_location_publisher = rospy.Publisher('/vehicle_location', String, queue_size=10)

        # Subscriber 초기화
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)

        # 최신 GPS 데이터 저장
        self.latest_gps_data = None

        # 1초마다 발행하는 타이머 설정
        rospy.Timer(rospy.Duration(1.0), self.publish_location)

        rospy.loginfo("GPS to Location Publisher node started")

    def gps_callback(self, msg):
        """
        /gps 토픽에서 위도와 경도를 읽어 저장합니다.
        """
        try:
            rospy.loginfo(f"Received GPS message: lat={msg.latitude}, lon={msg.longitude}")
            
            # JSON 데이터로 변환
            self.latest_gps_data = {
                "latitude": msg.latitude,
                "longitude": msg.longitude
            }
        
        except Exception as e:
            rospy.logerr(f"Error processing GPS data: {e}")

    def publish_location(self, event):
        """
        1초마다 /vehicle_location으로 GPS 좌표를 발행합니다.
        """
        if self.latest_gps_data:
            try:
                output_msg = String()
                output_msg.data = json.dumps(self.latest_gps_data)
                self.vehicle_location_publisher.publish(output_msg)
                rospy.loginfo(f"Published GPS Coordinates: {self.latest_gps_data}")
            
            except Exception as e:
                rospy.logerr(f"Error publishing GPS data: {e}")

if __name__ == '__main__':
    try:
        node = GPSToLocationPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("GPS to Location Publisher node terminated")
