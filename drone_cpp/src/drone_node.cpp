#include <rclcpp/rclcpp.hpp> ///< Include the ROS 2 library.
#include "drone_cpp/drone_node.hpp"///< Include the DroneNode class.
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

void DroneNode::handle_drop_done_request(///< Handle the request from the service.
    const std::shared_ptr<DropDroneSrv::Request> request,
    const std::shared_ptr<DropDroneSrv::Response> response)
{   
    
    
    mutex_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto right_neighbour_request = std::make_shared<SetAnchorAndNeighbours::Request>();///< Create a request for the right neighbor.
    right_neighbour_request->anchor = false;///< Set the anchor status to false.
    right_neighbour_request->new_neighbour = neighbour_left_;///< Set the neighbor id.
    right_neighbour_request->which_side = false;///< Set the side of the neighbor.
    std::string client_right = "set_anchor_and_neighbours" + std::to_string(neighbour_right_);
    auto sync_client_right= this->create_client<SetAnchorAndNeighbours>(
        client_right,               // service name
        rmw_qos_profile_services_default,  // qos profile
        mutex_group_                       // callback group
    );
    auto result_future_right = sync_client_right->async_send_request(right_neighbour_request,
    std::bind(&DroneNode::set_anchor_and_neighbours_response_callback, this, std::placeholders::_1));
    // auto result_future_right = sync_client_right->async_send_request(right_neighbour_request);

    auto left_neighbour_request = std::make_shared<SetAnchorAndNeighbours::Request>();///< Create a request for the left neighbor.
    left_neighbour_request->anchor = true;///< Set the anchor status to false.
    left_neighbour_request->new_neighbour = neighbour_right_;///< Set the neighbor id.
    left_neighbour_request->which_side = true;///< Set the side of the neighbor.
    std::string client_left = "set_anchor_and_neighbours" + std::to_string(neighbour_left_);

    auto sync_client_left= this->create_client<SetAnchorAndNeighbours>(
        client_left,               // service name
        rmw_qos_profile_services_default,  // qos profile
        mutex_group_                       // callback group
    );
    auto result_future_left = sync_client_left->async_send_request(left_neighbour_request,
    std::bind(&DroneNode::set_anchor_and_neighbours_response_callback, this, std::placeholders::_1));
    // auto result_future_left = sync_client_left->async_send_request(left_neighbour_request);
    land_ = true;///< Set the land status to true.


    

    auto req= request->drop;///< Get the request.
    response->dropped = true;///< Set the response to true.
    RCLCPP_INFO(this->get_logger(), "Drone %d dropped", id_);///< Log the drone drop.
}



void DroneNode::handle_anchor_and_neighbours_request(///< Handle the request from the service.
    const std::shared_ptr<SetAnchorAndNeighbours::Request> request,
    const std::shared_ptr<SetAnchorAndNeighbours::Response> response)
{
    bool anchor_bool = request->anchor;///< Set the anchor status.
    int neighbour_id = request->new_neighbour;///< Set the neighbor id.
    bool which_side = request->which_side;///< Set the side of the neighbor.

    RCLCPP_INFO(this->get_logger(), "Previous state");///< Log the anchor and neighbors set.
    RCLCPP_INFO(this->get_logger(), "Drone %d anchor %d neighbour_left %d neighbour_right %d", id_, anchor_, neighbour_left_, neighbour_right_);
    mutex_group_3_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    if(anchor_bool)///< If the drone is an anchor.
    {
        anchor_ = true;///< Set the anchor status to true.
        if(!which_side)///< If the neighbor is on the left.
        {
            neighbour_left_ = neighbour_id;///< Set the left neighbor id.
        }
        else///< If the neighbor is on the right.
        {
            neighbour_right_ = neighbour_id;///< Set the right neighbor id.
        }
        RCLCPP_INFO(this->get_logger(), "New state");///< Log the anchor and neighbors set.
        RCLCPP_INFO(this->get_logger(), "Drone %d anchor %d neighbour_left %d neighbour_right %d", id_, anchor_, neighbour_left_, neighbour_right_);


        // math
        int n=4;
        double inc= 360.0/(n-1);
        double targ;
        if((phase_angel_ +inc)>360)
        {
            targ = phase_angel_ + inc - 360;
        }
        else
        {
            targ = phase_angel_ + inc;
        }
        target_phase_angel_ = phase_angel_;
        
        

        RCLCPP_INFO(this->get_logger(), "Increment{%f}",inc);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Target angle{%f}",targ);///< Log the received angle.

        //
        auto right_neighbour_request = std::make_shared<CalcAngle::Request>();///< Create a request for the left neighbor.
        right_neighbour_request-> target_angle = targ;///< Set the angle.
        right_neighbour_request-> increment = inc;///< Set the radius.
        std::string service_name_calc_angle= "calc_angle_" + std::to_string(neighbour_right_);

        auto sync_client_right= this->create_client<CalcAngle>(
             service_name_calc_angle,               // service name
             rmw_qos_profile_services_default,  // qos profile
             mutex_group_3_                       // callback group
         );
        auto result_future_right = sync_client_right->async_send_request(right_neighbour_request,std::bind(&DroneNode::calc_angle_response_callback, this, std::placeholders::_1));
        // auto result_future_right = sync_client_right->async_send_request(right_neighbour_request);
        RCLCPP_INFO(this->get_logger(), "Calc angle service request sent from anchor{%d} to neighbour right{%d}",id_, neighbour_right_);///< Log the received angle.
        
    //     auto sync_client_left = this->create_client<SetAnchorAndNeighbours>(service_name_calc_angle);///< Create a client for the left neighbor.
    // // auto result_left = client_left_->async_send_request(left_neighbour_request);///< Send the request to the left neighbor.
    //     auto result_future_left = sync_client_left->async_send_request(left_neighbour_request);
    //     RCLCPP_INFO(this->get_logger(), "left neigbour %d", neighbour_left_);///< Log the received angle.
    //     RCLCPP_INFO(this->get_logger(), "Start calc at anchor");///< Log the received angle.
    }
    else///< If the drone is not an anchor.
    {
        if(!which_side)///< If the neighbor is on the left.
        {
            neighbour_left_ = neighbour_id;///< Set the left neighbor id.
        }
        else///< If the neighbor is on the right.
        {
            neighbour_right_ = neighbour_id;///< Set the right neighbor id.
        }
        RCLCPP_INFO(this->get_logger(), "New state");///< Log the anchor and neighbors set.
        RCLCPP_INFO(this->get_logger(), "Drone %d anchor %d neighbour_left %d neighbour_right %d", id_, anchor_, neighbour_left_, neighbour_right_);
    }
    
    response->done = true;///< Set the response to true.
    // RCLCPP_INFO(this->get_logger(), "Drone %d set anchor and neighbours", id_);///< Log the anchor and neighbors set.
}

void DroneNode::handle_calc_angle(///< Handle the request from the service.
    const std::shared_ptr<CalcAngle::Request> request,
    const std::shared_ptr<CalcAngle::Response> response)
{
    mutex_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    if(!anchor_)
    {
        double target_angle = request->target_angle;///< Get the target angle.
        double inc = request->increment;///< Get the increment.
        target_phase_angel_ = target_angle;///< Set the target angle.
        //Print the target_phase_angel_
        RCLCPP_INFO(this->get_logger(), "Drone %d target angle %f", id_, target_phase_angel_);///< Log the received angle.
        double diff_angle = abs(target_angle - phase_angel_);///< Calculate the difference in angle.
        if (diff_angle > 180) {
            diff_angle = 360 - diff_angle;
        }
        double targ;
        if((target_angle +inc)>360)
        {
            targ = target_angle + inc - 360;
        }
        else
        {
            targ = target_angle + inc;
        }
        double inc_rad = diff_angle * M_PI / 180.0;
        temp_alpha_ = (M_PI - inc_rad)/2;
        double r_2 = radius_ * sqrt(2 * (1-cos(inc_rad)));
        temp_target_x_ =  r_2 * cos(temp_alpha_);
        temp_target_y_ =  r_2 * sin(-temp_alpha_);


        RCLCPP_INFO(this->get_logger(), "Angle has been calculated %d", id_);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Drone %d target angle %f", id_, target_angle);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Inc %f", diff_angle);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Alpha %f", temp_alpha_*180/M_PI);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "R_2 %f", r_2);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Drone %d target x %f", id_, temp_target_x_);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Drone %d target y %f", id_, temp_target_y_);///< Log the received angle.
        auto right_neighbour_request = std::make_shared<CalcAngle::Request>();///< Create a request for the left neighbor.
        right_neighbour_request-> target_angle = targ;///< Set the angle.
        right_neighbour_request-> increment = inc;///< Set the radius.
        std::string service_name_calc_angle= "calc_angle_" + std::to_string(neighbour_right_);

        auto sync_client_right= this->create_client<CalcAngle>(
             service_name_calc_angle,               // service name
             rmw_qos_profile_services_default,  // qos profile
             mutex_group_2_                       // callback group
         );
        auto result_future_right = sync_client_right->async_send_request(right_neighbour_request,std::bind(&DroneNode::calc_angle_response_callback, this, std::placeholders::_1));
        // auto result_future_right = sync_client_right->async_send_request(right_neighbour_request);
        
        RCLCPP_INFO(this->get_logger(), "Calc angle request sent %d", neighbour_right_);///< Log the received angle.
    response->received = true;///< Set the response to true.
    // RCLCPP_INFO(this->get_logger(), "Drone %d received angle", id_);///< Log the received angle.
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "End calc at Anchor{%d}",id_);///< Log the received angle.
        RCLCPP_INFO(this->get_logger(), "Drone %d target angle %f", id_, target_phase_angel_);///< Log the received angle.
    }
    
}

void DroneNode::handle_move_drone_request(///< Handle the request from the service.
    const std::shared_ptr<MoveDroneSrv::Request> request,
    const std::shared_ptr<MoveDroneSrv::Response> response)
{   
    target_x_= temp_target_x_;///< Set the target x position.
    target_y_= temp_target_y_;///< Set the target y position.
    alpha_=temp_alpha_;///< Set the alpha.

    

    auto req= request->move;///< Get the request.
    response->moved = true;///< Set the response to true.
    // RCLCPP_INFO(this->get_logger(), "Drone %d dropped", id_);///< Log the drone drop.
}

void DroneNode::set_anchor_and_neighbours_response_callback(
    rclcpp::Client<SetAnchorAndNeighbours>::SharedFuture future) {
  auto status = future.wait_for(0.5s);
  if (status == std::future_status::ready) {
    auto result = static_cast<int>(future.get()->done);
    // print_profile(result);
  }
}

void DroneNode::calc_angle_response_callback(
    rclcpp::Client<CalcAngle>::SharedFuture future) {
  auto status = future.wait_for(0.5s);
  if (status == std::future_status::ready) {
    auto result = static_cast<int>(future.get()->received);
    // print_profile(result);
  }
}

void DroneNode::callback_left(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg)///< Callback function for the left neighbor.
{
    phase_angel_left_ = msg->phase_angle.data;///< Get the phase angle of the left neighbor.
    radius_left_ = msg->radius.data;///< Get the radius of the left neighbor.
}
void DroneNode::callback_right(const drone_swarm_msgs::msg::DroneStatus::SharedPtr msg)///< Callback function for the right neighbor.
{
    phase_angel_right_ = msg->phase_angle.data;///< Get the phase angle of the right neighbor.
    radius_right_ = msg->radius.data;///< Get the radius of the right neighbor.
}

void DroneNode::timer_callback()///< Timer callback function.
{
    status_msg_.id.data = id_;///< Set the drone id.
    status_msg_.anchor.data = anchor_;///< Set the anchor status.
    status_msg_.phase_angle.data = static_cast<float>(phase_angel_);///< Set the phase angle.
    status_msg_.radius.data = static_cast<float>(radius_);///< Set the radius.
    status_msg_.neighbor_left.data = neighbour_left_;///< Set the left neighbor id.
    status_msg_.neighbor_right.data = neighbour_right_;///< Set the right neighbor id.
    publisher_->publish(status_msg_);///< Publish the status message.
}
void DroneNode::Move_drone_callback()///< Move drone callback function.
{
    move_msg_.target_pos_x.data = target_x_;///< Set the target x position.
    move_msg_.target_pos_y.data = target_y_;///< Set the target y position.
    move_msg_.alpha.data = alpha_;///< Set the alpha.
    move_msg_.land.data = land_;///< Set the land status.
    publisher_move_->publish(move_msg_);///< Publish the move message.
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);///< Initialize the ROS 2 system.
    auto drone_node = std::make_shared<DroneNode>("drone_number");///< Create a DroneNode.
    rclcpp::spin(drone_node);///< Spin the node to continuously publish messages.
    rclcpp::shutdown();//< Shutdown the ROS 2 system.
}