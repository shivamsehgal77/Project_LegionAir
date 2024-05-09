#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <drone_swarm_msgs/msg/drone_status.hpp>
#include <drone_swarm_msgs/srv/drop_drone.hpp>
#include <drone_swarm_msgs/srv/set_anchor_and_neighbours.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp" ///< Include the rcl_interfaces library

using DropDroneSrv = drone_swarm_msgs::srv::DropDrone;
using SetAnchorAndNeighbours = drone_swarm_msgs::srv::SetAnchorAndNeighbours;
class DroneNode : public rclcpp::Node
{
    public:
        DroneNode(std::string node_name): Node(node_name) {

        //----------------------------------------------
        // Parameters
        //----------------------------------------------
        rcl_interfaces::msg::ParameterDescriptor descriptor_id;
        descriptor_id.description = "Drone ID";
        descriptor_id.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("id", 0, descriptor_id);
        id_ = this->get_parameter("id").as_int();

        rcl_interfaces::msg::ParameterDescriptor descriptor_anchor;
        descriptor_anchor.description = "Anchor";
        descriptor_anchor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        this->declare_parameter("anchor", false, descriptor_anchor);
        anchor_ = this->get_parameter("anchor").as_bool();

        rcl_interfaces::msg::ParameterDescriptor descriptor_phase_angle;
        descriptor_phase_angle.description = "Phase Angle";
        descriptor_phase_angle.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("phase_angle", 0.0, descriptor_phase_angle);
        phase_angel_ = this->get_parameter("phase_angle").as_double();

        rcl_interfaces::msg::ParameterDescriptor descriptor_radius;
        descriptor_radius.description = "Radius";
        descriptor_radius.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("radius", 1.0, descriptor_radius);
        radius_ = this->get_parameter("radius").as_double();

        rcl_interfaces::msg::ParameterDescriptor descriptor_neighbour_left;
        descriptor_neighbour_left.description = "Neighbor Left";
        descriptor_neighbour_left.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("neighbour_left", 2, descriptor_neighbour_left);
        neighbour_left_ = this->get_parameter("neighbour_left").as_int();

        rcl_interfaces::msg::ParameterDescriptor descriptor_neighbour_right;
        descriptor_neighbour_right.description = "Neighbour Right";
        descriptor_neighbour_right.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("neighbour_right", 1, descriptor_neighbour_right);
        neighbour_right_ = this->get_parameter("neighbour_right").as_int();

        status_msg_=drone_swarm_msgs::msg::DroneStatus();
        std::string topic_name = "drone_status_" + std::to_string(id_);
        publisher_ = this->create_publisher<drone_swarm_msgs::msg::DroneStatus>(topic_name, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DroneNode::timer_callback, this));
        std::string topic_name_left = "drone_status_" + std::to_string(neighbour_left_);
        std::string topic_name_right = "drone_status_" + std::to_string(neighbour_right_);  
        subscriber_left_ = this->create_subscription<drone_swarm_msgs::msg::DroneStatus>(topic_name_left, 10, std::bind(&DroneNode::callback_left, this, std::placeholders::_1));
        subscriber_right_ = this->create_subscription<drone_swarm_msgs::msg::DroneStatus>(topic_name_right, 10, std::bind(&DroneNode::callback_right, this, std::placeholders::_1));


        std::string service_name_drop_drone_ = "drop_drone_" + std::to_string(id_);
        service_1_ = this->create_service<DropDroneSrv>(
        service_name_drop_drone_,
        std::bind(&DroneNode::handle_drop_done_request,
                  this, std::placeholders::_1, std::placeholders::_2));

        // Set anchor and neighbors service
        std::string service_name_anch_neigh = "set_anchor_and_neighbours" + std::to_string(id_);
        service_2_ = this->create_service<SetAnchorAndNeighbours>(
        service_name_anch_neigh,
        std::bind(&DroneNode::handle_anchor_and_neighbours_request,
                  this, std::placeholders::_1, std::placeholders::_2));

        }
    private:

    void handle_drop_done_request(
      const std::shared_ptr<DropDroneSrv::Request> request,
      std::shared_ptr<DropDroneSrv::Response> response);
    // Service object
    rclcpp::Service<DropDroneSrv>::SharedPtr service_1_;

    void handle_anchor_and_neighbours_request(
      const std::shared_ptr<SetAnchorAndNeighbours::Request> request,
      std::shared_ptr<SetAnchorAndNeighbours::Response> response);
    // Service object
    rclcpp::Service<SetAnchorAndNeighbours>::SharedPtr service_2_;

    void set_anchor_and_neighbours_response_callback(
    rclcpp::Client<SetAnchorAndNeighbours>::SharedFuture future);

    rclcpp::CallbackGroup::SharedPtr mutex_group_;

    drone_swarm_msgs::msg::DroneStatus status_msg_;
    rclcpp::Publisher<drone_swarm_msgs::msg::DroneStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<drone_swarm_msgs::msg::DroneStatus>::SharedPtr subscriber_left_;
    rclcpp::Subscription<drone_swarm_msgs::msg::DroneStatus>::SharedPtr subscriber_right_;
    void callback_left(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg);
    void callback_right(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg);
    void timer_callback();
    bool anchor_;
    int id_ ;
    double phase_angel_ ; /// deg
    double radius_ ; /// m
    int neighbour_left_ ;
    int neighbour_right_ ;

    // Left
    double phase_angel_left_ ; /// deg
    double radius_left_ ; /// m

    // Right
    double phase_angel_right_ ; /// deg
    double radius_right_ ; /// m



};