#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster_node.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

struct ExtrinsicTransform {
    std::string parent;
    std::string child;
    std::vector<double> T_child_wrt_parent;
    std::vector<double> RPY_parent_to_child;
};

void publish_static_transforms(const std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& static_broadcaster, const rclcpp::Node::SharedPtr& node) {
    std::vector<ExtrinsicTransform> extrinsics = {
        {"body", "world", {0.068, 0.0116, 0.0168}, {0, 270, 0}},
        {"body", "hires", {0.068, -0.012, 0.015}, {90, 270, 0}}
    };

    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    for (const auto& extrinsic : extrinsics) {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = node->now();
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
        static_transform.transform.rotation.x = q.x();
        static_transform.transform.rotation.y = q.y();
        static_transform.transform.rotation.z = q.z();
        static_transform.transform.rotation.w = q.w();

        static_transforms.push_back(static_transform);
    }
    static_broadcaster->sendTransform(static_transforms);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tf_publisher_cpp_node");
    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [=]() {
        publish_static_transforms(static_broadcaster, node);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
