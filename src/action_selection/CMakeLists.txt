cmake_minimum_required(VERSION 2.8.3)
project(action_selection)

find_package(catkin REQUIRED COMPONENTS
  basic_skills
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES action_selection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
