cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_ros)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs geometry_msgs
)
catkin_python_setup()

catkin_install_python(PROGRAMS src/kalman_filter_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


