cmake_minimum_required(VERSION 2.8.3)
project(skill_execution)

find_package(catkin REQUIRED COMPONENTS
  rospy
  basic_skills
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES skill_execution
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
