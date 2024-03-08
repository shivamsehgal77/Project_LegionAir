/**
 * @file tf_publisher_cpp_node.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief The following cpp file is the ros node for publishing the static transforms based on the given camera extrinsics of the drone in the form of a .conf file
 * @version 0.1
 * @date 2024-02-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class TfPublisherCppNode
{
public:
  TfPublisherCppNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    // Get the path of the .conf file
    std::string conf_file_path;
    pnh.getParam("conf_file_path", conf_file_path);

    // Read the .conf file
    std::ifstream conf_file(conf_file_path);
    if (!conf_file.is_open())
    {
      ROS_ERROR("Unable to open the .conf file");
      return;
    }

    std::string line;
    while (std::getline(conf_file, line))
    {
      std::vector<std::string> tokens;
      std::string token;
      std::istringstream tokenStream(line);
      while (std::getline(tokenStream, token, ' '))
      {
        tokens.push_back(token);
      }

      if (tokens.size() != 7)
      {
        ROS_ERROR("Invalid line in the .conf file");
        continue;
      }

      std::string child_frame_id = tokens[0];
      std::string parent_frame_id = tokens[1];
      double x = std::stod(tokens[2]);
      double y = std::stod(tokens[3]);
      double z = std::stod(tokens[4]);
      double roll = std::stod(tokens[5]);
      double pitch = std::stod(tokens[6]);
      double yaw = std::stod(tokens[7]);

      // Create the transform
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = parent_frame_id;
      transformStamped.child_frame_id = child_frame_id;
      transformStamped.transform.translation.x = x;
      transformStamped.transform.translation.y = y;
      transformStamped.transform.translation.z = z;
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w;

      // Publish the transform
      static_tf_broadcaster.sendTransform(transformStamped);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publisher_cpp_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TfPublisherCppNode tf_publisher_cpp_node(nh, pnh);

  ros::spin();
  return 0;
}
 