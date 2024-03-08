cmake_minimum_required(VERSION 3.0.2)
project(your_tf_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_publisher_cpp_node src/tf_publisher_cpp_node.cpp)

target_link_libraries(tf_publisher_cpp_node
  ${catkin_LIBRARIES}
)
