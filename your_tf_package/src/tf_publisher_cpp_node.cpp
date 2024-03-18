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
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

struct ExtrinsicTransform {
    std::string parent;
    std::string child;
    std::vector<double> T_child_wrt_parent;
    std::vector<double> RPY_parent_to_child;
};

void publish_static_transforms() {
    // Create a static transform broadcaster
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<ExtrinsicTransform> extrinsics = {
        {"body", "world", {0.068, 0.0116, 0.0168}, {0, 270, 0}},
        {"body", "hires", {0.068, -0.012, 0.015}, {90, 270, 0}}
    };

    // Publish the static transforms
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    for (const auto& extrinsic : extrinsics) {
        geometry_msgs::TransformStamped static_transform;
        static_transform.header.stamp = ros::Time::now();
        static_transform.header.frame_id = extrinsic.parent;
        static_transform.child_frame_id = extrinsic.child;
        static_transform.transform.translation.x = extrinsic.T_child_wrt_parent[0];
        static_transform.transform.translation.y = extrinsic.T_child_wrt_parent[1];
        static_transform.transform.translation.z = extrinsic.T_child_wrt_parent[2];
        double roll = extrinsic.RPY_parent_to_child[0];
        double pitch = extrinsic.RPY_parent_to_child[1];
        double yaw = extrinsic.RPY_parent_to_child[2];
        double roll_rad = roll * M_PI / 180;
        double pitch_rad = pitch * M_PI / 180;
        double yaw_rad = yaw * M_PI / 180;
        Eigen::AngleAxisd rotation_vector_x(roll_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotation_vector_y(pitch_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rotation_vector_z(yaw_rad, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = rotation_vector_x * rotation_vector_y * rotation_vector_z;
        Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
        Eigen::Matrix3d rotation_matrix_inverse = rotation_matrix.inverse();
        Eigen::Quaterniond q_inverse(rotation_matrix_inverse);
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_transforms.push_back(static_transform);
    }
    static_broadcaster.sendTransform(static_transforms);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher_cpp_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    while (ros::ok()) {
        publish_static_transforms();
        rate.sleep();
    }
    return 0;
}
