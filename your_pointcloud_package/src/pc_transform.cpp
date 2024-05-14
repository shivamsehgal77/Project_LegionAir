#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <sched.h>
#include <cstring>
#include <string>
#include <stdexcept>
#include <pthread.h>
#include <cassert>
#include <fstream>
#include <thread>

enum class CPUS : size_t {
  CPU1,
  CPU2,
  CPU3,
  CPU4,
  CPU5,
  CPU6,
  CPU7,
  CPU8
};

std::runtime_error getError(std::string msg, int result) {
  return std::runtime_error(msg + " Reason:" + std::strerror(result));
}

void applyAffinity(const CPUS affCPU) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  const auto aff = static_cast<std::underlying_type<CPUS>::type>(affCPU);
  CPU_SET(aff, &cpuset);
  auto result = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (result != 0) {
    throw getError("Failed to attach affinity", result);
  }
}

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("pc_transformer") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    node_namespace_ = this->get_namespace();
    RCLCPP_INFO_STREAM(this->get_logger(), "Namespace in pc_transform: " << node_namespace_);
    //rclcpp::SubscriptionOptions subscription_options;
    //subscription_options.qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rcl_interfaces::msg::ParameterDescriptor descriptor_id;
    descriptor_id.description = "Drone ID";
    descriptor_id.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    if (!node_namespace_.empty() && node_namespace_[0] == '/') {
      param_namespace_ = node_namespace_.substr(1);
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter namespace in pc_transform: " << param_namespace_);
    }
    this->declare_parameter(param_namespace_+".id", 0, descriptor_id);
    id_ = this->get_parameter(param_namespace_+".id").as_int();
    RCLCPP_INFO_STREAM(this->get_logger(), "Drone ID in pc_transform node: " << id_);
    std::string topic_name_tof = node_namespace_ + "/tof_pc";
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name_tof, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(), std::bind(&PointCloudTransformer::pc_callback, this, std::placeholders::_1));
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rgb_pcl", rclcpp::SensorDataQoS());

    translation_vector = Eigen::Vector3d::Zero();
    q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    rotation_matrix = Eigen::Matrix3d::Zero();
    filtered_size = 0;
    remainder = 0;
    division = 0;
  }

private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2, 1.5);
    pass.filter(*cloud_filtered);

    RCLCPP_INFO(this->get_logger(), "Size of the filtered point cloud: %ld", cloud_filtered->size());

    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("hires", pc_msg->header.frame_id, tf2::TimePointZero);
      translation_vector[0] = transform_stamped.transform.translation.x;
      translation_vector[1] = transform_stamped.transform.translation.y;
      translation_vector[2] = transform_stamped.transform.translation.z;
      tf_quat[0] = transform_stamped.transform.rotation.x;
      tf_quat[1] = transform_stamped.transform.rotation.y;
      tf_quat[2] = transform_stamped.transform.rotation.z;
      tf_quat[3] = transform_stamped.transform.rotation.w;
      q = Eigen::Quaterniond(tf_quat[3], tf_quat[0], tf_quat[1], tf_quat[2]);
      rotation_matrix = q.toRotationMatrix();
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << translation_vector[0], translation_vector[1], translation_vector[2];
      transform.rotate(rotation_matrix.cast<float>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform);
      sensor_msgs::msg::PointCloud2 transformed_pc;
      pcl::toROSMsg(*transformed_cloud, transformed_pc);
      transformed_pc.header.frame_id = "hires";
      pc_pub_->publish(transformed_pc);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Failure: %s\n", ex.what());
      return;
    }
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  Eigen::Vector3d translation_vector;
  double tf_quat[4];
  Eigen::Quaterniond q;
  Eigen::Matrix3d rotation_matrix;
  size_t filtered_size;
  int remainder;
  int division;
  int id_;
  std::string node_namespace_;
  std::string param_namespace_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
