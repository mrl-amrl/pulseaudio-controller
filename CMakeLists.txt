cmake_minimum_required(VERSION 2.8.3)
project(pulseaudio_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Item.msg
)

add_service_files(
  FILES
  ListItems.srv
  SetItem.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
