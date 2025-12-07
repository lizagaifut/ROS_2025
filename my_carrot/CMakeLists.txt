cmake_minimum_required(VERSION 3.0.2)
project(my_carrot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  turtlesim
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rotating_carrot_broadcaster.py
  scripts/chase_rotating_carrot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
