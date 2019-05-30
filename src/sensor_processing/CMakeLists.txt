cmake_minimum_required(VERSION 2.8.3)
project(sensor_processing)

find_package(catkin REQUIRED COMPONENTS
  robocup_msgs
  geometry_msgs
  tf
  rospy
)
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_processing
#  CATKIN_DEPENDS robocup_msgs
#  DEPENDS system_lib
)

## Generate Protobuf messages
protobuf_generate_python(PROTO_PY
  proto/messages_robocup_ssl_detection.proto
  proto/messages_robocup_ssl_geometry.proto
  proto/messages_robocup_ssl_refbox_log.proto
  proto/messages_robocup_ssl_wrapper.proto
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
