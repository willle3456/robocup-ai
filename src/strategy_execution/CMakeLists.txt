cmake_minimum_required(VERSION 2.8.3)
project(strategy_execution)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  basic_skills
  skill_execution
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strategy_execution
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
