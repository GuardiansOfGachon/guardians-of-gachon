cmake_minimum_required(VERSION 3.0.2)
project(global_path_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## 메시지 파일
add_message_files(
  FILES
    global_data.msg
)

## 메시지 의존성
generate_messages(
  DEPENDENCIES
    std_msgs
)

## Catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    message_runtime
)

## 인클루드 디렉토리
include_directories(
  ${catkin_INCLUDE_DIRS}
)

