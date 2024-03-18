cmake_minimum_required(VERSION 2.8.3)
project(your_pointcloud_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
)

catkin_install_python(PROGRAMS src/pointcloud_transformer.py
                              src/pointcloud_transformer_rgb.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ executable
add_executable(pointcloud_transformer src/pc_transform.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pointcloud_transformer
  ${catkin_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)