cmake_minimum_required(VERSION 3.0.2)
project(your_tf_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS src/tf_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Declare a C++ executable
add_executable(tf_publisher_cpp src/tf_publisher_cpp_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tf_publisher_cpp
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)