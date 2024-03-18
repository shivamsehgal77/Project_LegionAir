cmake_minimum_required(VERSION 3.0.2)
project(your_tf_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_package()

catkin_install_python(PROGRAMS src/tf_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)