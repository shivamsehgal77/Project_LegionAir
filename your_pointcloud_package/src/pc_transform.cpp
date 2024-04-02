#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

#include <sched.h>
#include <cstring>
#include <string>
#include <stdexcept>
#include <pthread.h>
#include  <stddef.h>
#include  <assert.h>
#include <fstream>
#include <thread>

enum class CPUS:size_t{
CPU1,
CPU2,
CPU3,
CPU4,
CPU5,
CPU6,
CPU7,
CPU8
};

std::runtime_error getError(std::string msg,int result){
    return std::runtime_error(msg+" Reason:"+std::string(std::strerror(result)));
}


void applyAffinity(const CPUS affCPU) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    const auto aff = static_cast<std::underlying_type<CPUS>::type>(affCPU);
    CPU_SET(aff, &cpuset);  
    auto result = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        throw getError("Failed to attach affinity",result);
    }
}

class PointCloudTransformer {
public:
    PointCloudTransformer() {
        ros::NodeHandle nh;
        tf_buffer_ = new tf2_ros::Buffer();
        tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
        pc_sub_ = nh.subscribe("/tof_pc", 1, &PointCloudTransformer::pc_callback, this);
        pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgb_pcl", 1);
        transformed_points = Eigen::MatrixXd(3, 38528);
        init_points = Eigen::MatrixXd(3, 38528);
        points1 = Eigen::MatrixXd(3, 4816);
        points2 = Eigen::MatrixXd(3, 4816);
        points3 = Eigen::MatrixXd(3, 4816);
        points4 = Eigen::MatrixXd(3, 4816);
        points5 = Eigen::MatrixXd(3, 4816);
        points6 = Eigen::MatrixXd(3, 4816);
        points7 = Eigen::MatrixXd(3, 4816);
        points8 = Eigen::MatrixXd(3, 4816);
        translation_vector = Eigen::Vector3d::Zero();
        tf_quat[4] = {0.0};
        q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        rotation_matrix = Eigen::Matrix3d::Zero();
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
        pcl::fromROSMsg(*pc_msg, init_cloud);
        // Extract the points from the point cloud message
        size_t column_count = init_cloud.size();
        for (size_t i = 0; i < column_count; ++i) {
            init_points(0, i) = init_cloud.points[i].x;
            init_points(1, i) = init_cloud.points[i].y;
            init_points(2, i) = init_cloud.points[i].z;
        }
        // Split the points into 8 different vectors of points
        points1 = init_points.block(0, 0, 3, column_count/8);
        points2 = init_points.block(0, column_count/8, 3, column_count/8);
        points3 = init_points.block(0, column_count/4, 3, column_count/8);
        points4 = init_points.block(0, 3*column_count/8, 3, column_count/8);
        points5 = init_points.block(0, column_count/2, 3, column_count/8);
        points6 = init_points.block(0, 5*column_count/8, 3, column_count/8);
        points7 = init_points.block(0, 3*column_count/4, 3, column_count/8);
        points8 = init_points.block(0, 7*column_count/8, 3, column_count/8);
        sensor_msgs::PointCloud2 transformed_pc;
        // Look up the transform from the world frame to the rgb frame
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform("hires", pc_msg->header.frame_id, ros::Time(0));
            // Use the transform_stamped to transform the point cloud on separate threads
            // by using the 4 different vectors of points
            translation_vector[0] = transform_stamped.transform.translation.x;
            translation_vector[1] = transform_stamped.transform.translation.y;
            translation_vector[2] = transform_stamped.transform.translation.z;
            tf_quat[0] = transform_stamped.transform.rotation.x;
            tf_quat[1] = transform_stamped.transform.rotation.y;
            tf_quat[2] = transform_stamped.transform.rotation.z;
            tf_quat[3] = transform_stamped.transform.rotation.w;
            q = Eigen::Quaterniond(tf_quat[3], tf_quat[0], tf_quat[1], tf_quat[2]);
            rotation_matrix = q.toRotationMatrix();
            std::thread t1([this](){
                transformed_points.block(0, 0, 3, points1.cols()) = rotation_matrix * points1 + translation_vector.replicate(1, points1.cols());
            });
            std::thread t2([this](){
                transformed_points.block(0, points1.cols(), 3, points2.cols()) = rotation_matrix * points2 + translation_vector.replicate(1, points2.cols());
            });
            std::thread t3([this](){
                transformed_points.block(0, 2*points1.cols(), 3, points3.cols()) = rotation_matrix * points3 + translation_vector.replicate(1, points3.cols());
            });
            std::thread t4([this](){
                transformed_points.block(0, 3*points1.cols(), 3, points4.cols()) = rotation_matrix * points4 + translation_vector.replicate(1, points4.cols());
            });
            std::thread t5([this](){
                transformed_points.block(0, 4*points1.cols(), 3, points5.cols()) = rotation_matrix * points5 + translation_vector.replicate(1, points5.cols());
            });
            std::thread t6([this](){
                transformed_points.block(0, 5*points1.cols(), 3, points6.cols()) = rotation_matrix * points6 + translation_vector.replicate(1, points6.cols());
            });
            std::thread t7([this](){
                transformed_points.block(0, 6*points1.cols(), 3, points7.cols()) = rotation_matrix * points7 + translation_vector.replicate(1, points7.cols());
            });
            std::thread t8([this](){
                transformed_points.block(0, 7*points1.cols(), 3, points8.cols()) = rotation_matrix * points8 + translation_vector.replicate(1, points8.cols());
            });
            t1.join();
            t2.join();
            t3.join();
            t4.join();
            t5.join();
            t6.join();
            t7.join();
            t8.join();
            // Create a new point cloud message to store the transformed points
            transformed_cloud.width = transformed_points.cols();
            transformed_cloud.height = 1;
            transformed_cloud.is_dense = false;
            transformed_cloud.points.resize(transformed_cloud.width * transformed_cloud.height);
            std::cout << "Size of transformed points: " << transformed_points.rows() << " x " << transformed_points.cols() << std::endl;
            for (size_t i = 0; i < transformed_cloud.size(); ++i) {
                transformed_cloud.points[i].x = transformed_points(0, i);
                transformed_cloud.points[i].y = transformed_points(1, i);
                transformed_cloud.points[i].z = transformed_points(2, i);
            }
            pcl::toROSMsg(transformed_cloud, transformed_pc);
            transformed_pc.header.frame_id = "hires";                                 

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure, I am here %s\n", ex.what());
            ROS_WARN("%s", ex.what());
            return;
        }
        // Transform the point cloud by using transform_stamped and the point cloud message
        // tf2::doTransform(*pc_msg, transformed_pc, transform_stamped); // Use tf2::doTransform instead of tf2_sensor_msgs::doTransform
        pc_pub_.publish(transformed_pc);
    }

private:
    tf2_ros::Buffer* tf_buffer_;
    
    tf2_ros::TransformListener* tf_listener_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    Eigen::MatrixXd init_points;
    Eigen::MatrixXd transformed_points;
    Eigen::MatrixXd points1;
    Eigen::MatrixXd points2;
    Eigen::MatrixXd points3;
    Eigen::MatrixXd points4;
    Eigen::MatrixXd points5;
    Eigen::MatrixXd points6;
    Eigen::MatrixXd points7;
    Eigen::MatrixXd points8;
    double tf_quat[4];
    Eigen::Quaterniond q;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_vector;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointCloud<pcl::PointXYZ> init_cloud;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_transform");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}
