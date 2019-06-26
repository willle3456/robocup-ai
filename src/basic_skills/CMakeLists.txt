cmake_minimum_required(VERSION 2.8.3)
project(basic_skills)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES basic_skills
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TEST)
  find_package(rostest REQUIRED)
  add_rostest(test/utilities/utilities.test)
  add_rostest(test/planner_helpers/planner_helpers.test)
  add_rostest(test/skills/skills.test)
  add_rostest(test/algorithms/algorithms.test)
endif()
