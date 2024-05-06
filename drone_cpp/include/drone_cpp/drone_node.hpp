#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <drone_swarm_msgs/msg/drone_status.hpp>

class DroneNode : public rclcpp::Node
{
    public:
        DroneNode(std::string node_name): Node(node_name), anchor_{false}, id_{0}, phase_angel_{0.0}, radius_{1.0}, neighbor_left_{0}, neighbor_right_{0} {
            status_msg_=drone_swarm_msgs::msg::DroneStatus();
            publisher_ = this->create_publisher<drone_swarm_msgs::msg::DroneStatus>("drone_status", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DroneNode::timer_callback, this));

        }
    private:
    drone_swarm_msgs::msg::DroneStatus status_msg_;
    rclcpp::Publisher<drone_swarm_msgs::msg::DroneStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    bool anchor_;
    int id_ ;
    float phase_angel_ ; /// deg
    float radius_ ; /// m
    int neighbor_left_ ;
    int neighbor_right_ ;

};