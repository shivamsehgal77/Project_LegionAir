#include <rclcpp/rclcpp.hpp> ///< Include the ROS 2 library.
#include "drone_cpp/drone_node.hpp"///< Include the DroneNode class.

void DroneNode::timer_callback()///< Timer callback function.
{
    status_msg_.id.data = id_;///< Set the drone id.
    status_msg_.anchor.data = anchor_;///< Set the anchor status.
    status_msg_.phase_angle.data = phase_angel_;///< Set the phase angle.
    status_msg_.radius.data = radius_;///< Set the radius.
    status_msg_.neighbor_left.data = neighbor_left_;///< Set the left neighbor id.
    status_msg_.neighbor_right.data = neighbor_right_;///< Set the right neighbor id.
    publisher_->publish(status_msg_);///< Publish the status message.
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);///< Initialize the ROS 2 system.
    auto drone_node = std::make_shared<DroneNode>("drone_one");///< Create a DroneNode.
    rclcpp::spin(drone_node);///< Spin the node to continuously publish messages.
    rclcpp::shutdown();//< Shutdown the ROS 2 system.
}