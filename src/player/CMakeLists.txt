cmake_minimum_required(VERSION 2.8.3)
project(player)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robocup_msgs
  geometry_msgs
  action_selection
  strategy_execution
)

catkin_python_setup()

catkin_package(
  #CATKIN_DEPENDS strategy_selection rospy std_msgs message_runtime geometry_msgs coach robocup_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/player_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
