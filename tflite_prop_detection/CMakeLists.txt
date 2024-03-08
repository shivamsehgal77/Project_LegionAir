cmake_minimum_required(VERSION 3.0.2)
project(tflite_prop_detection)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -ffast-math)
endif()
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  voxl_mpa_to_ros
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs voxl_mpa_to_ros cv_bridge image_transport
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tflite_prop_detection_cpp src/tflite_prop_detection_cpp_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tflite_prop_detection_cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_python_setup()

catkin_install_python(PROGRAMS src/tflite_prop_detection_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

