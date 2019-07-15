cmake_minimum_required(VERSION 2.8.3)
project(path_planning_rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  robocup_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planning_rqt
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

install(PROGRAMS
  scripts/path_planning_rqt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
