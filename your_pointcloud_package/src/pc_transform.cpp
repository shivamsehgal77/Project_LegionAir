#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

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
        column_count = 0;
        init_points.resize(3, 38528);
        transformed_points.resize(3, 38528);
        points1.resize(3, 9632);
        points2.resize(3, 9632);
        points3.resize(3, 9632);
        points4.resize(3, 9632);
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
        // Store the pc_msg in a member variable
        column_count = 0;

        
        pcl::fromROSMsg(*pc_msg, cloud);
        
        for (size_t i = 0; i < cloud.size(); ++i) {
            init_points(0, column_count) = cloud.points[i].x;
            init_points(1, column_count) = cloud.points[i].y;
            init_points(2, column_count) = cloud.points[i].z;
            column_count++;
        }
        std::cout << "Size of points: " << init_points.rows() << " x " << init_points.cols() << std::endl;
        // Divide the points eigen matrix of (3, column_count) into 4 different vectors by slicing
        // the matrix in the shape of  (3, column_count/4)
        points1 = init_points.block(0, 0, 3, column_count/4);
        points2 = init_points.block(0, column_count/4, 3, column_count/4);
        points3 = init_points.block(0, column_count/2, 3, column_count/4);
        points4 = init_points.block(0, 3*column_count/4, 3, column_count/4);
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
            // Do the rotation and translation steps for 4 different vectors of points on separate cpu threads and assign enum CPU4, CPU5, CPU6, CPU7
            std::thread t1([&](){
                applyAffinity(CPUS::CPU5);
                // Apply the rotation and translation to the points by doing column wise matrix multiplication
                for (size_t i = 0; i < points1.cols(); ++i) {
                    transformed_points.col(i) = rotation_matrix * points1.col(i) + translation_vector;
                }
            });
            std::thread t2([&](){
                applyAffinity(CPUS::CPU6);
                for (size_t i = 0; i < points2.cols(); ++i) {
                    transformed_points.col(i) = rotation_matrix * points2.col(i) + translation_vector;
                }
            });
            std::thread t3([&](){
                applyAffinity(CPUS::CPU7);
                for (size_t i = 0; i < points3.cols(); ++i) {
                    transformed_points.col(i) = rotation_matrix * points3.col(i) + translation_vector;
                }
            });
            std::thread t4([&](){
                applyAffinity(CPUS::CPU8);
                for (size_t i = 0; i < points4.cols(); ++i) {
                    transformed_points.col(i) = rotation_matrix * points4.col(i) + translation_vector;
                }
            });
            t1.join();
            t2.join();
            t3.join();
            t4.join();
            // Convert the transformed_points to a point cloud message
            
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
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf2_ros::TransformListener* tf_listener_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    Eigen::MatrixXd init_points;
    Eigen::MatrixXd points1;
    Eigen::MatrixXd points2;
    Eigen::MatrixXd points3;
    Eigen::MatrixXd points4;
    Eigen::MatrixXd transformed_points;
    double tf_quat[4];
    Eigen::Quaterniond q;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_vector;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    int column_count;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_transform");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}
