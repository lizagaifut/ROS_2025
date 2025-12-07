cmake_minimum_required(VERSION 3.0.2)
project(carrot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  turtlesim
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/turtle_tf_broadcaster.py
  scripts/turtle_tf_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
