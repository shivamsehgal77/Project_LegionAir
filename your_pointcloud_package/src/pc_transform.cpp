#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
        // transformed_points = Eigen::MatrixXd::Zero(3, 38528);
        // init_points = Eigen::MatrixXd::Zero(3, 38528);
        // points1 = Eigen::MatrixXd::Zero(3, 7500);
        // points2 = Eigen::MatrixXd::Zero(3, 7500);
        // points3 = Eigen::MatrixXd::Zero(3, 7500);
        // points4 = Eigen::MatrixXd::Zero(3, 7500);
        // points5 = Eigen::MatrixXd::Zero(3, 7500);
        // points6 = Eigen::MatrixXd::Zero(3, 7500);
        // points7 = Eigen::MatrixXd::Zero(3, 7500);
        // points8 = Eigen::MatrixXd::Zero(3, 7500);
        translation_vector = Eigen::Vector3d::Zero();
        tf_quat[4] = {0.0};
        q = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        rotation_matrix = Eigen::Matrix3d::Zero();
        filtered_size = 0;
        remainder = 0;
        division = 0;
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.2, 1.5);
        pass.filter(*cloud_filtered);
        // Print the size of the cloud_filtered point cloud
        ROS_INFO("Size of the filtered point cloud: %ld", cloud_filtered->size());
        try {
            geometry_msgs::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform("hires", pc_msg->header.frame_id, ros::Time(0));
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
                sensor_msgs::PointCloud2 transformed_pc;
                pcl::toROSMsg(*transformed_cloud, transformed_pc);
                transformed_pc.header.frame_id = "hires";
                pc_pub_.publish(transformed_pc);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Failure, I am here %s\n", ex.what());
            ROS_WARN("%s", ex.what());
            return;
        }
        
        
        // sensor_msgs::PointCloud2 transformed_pc;
        // pcl::toROSMsg(*cloud_filtered, transformed_pc);
        // transformed_pc.header.frame_id = "world";
        // pc_pub_.publish(transformed_pc);
        // filtered_size = 0;
        // sensor_msgs::PointCloud2 p_in = *pc_msg;
        // sensor_msgs::PointCloud2ConstIterator<float> x_in(p_in, "x");
        // sensor_msgs::PointCloud2ConstIterator<float> y_in(p_in, "y");
        // sensor_msgs::PointCloud2ConstIterator<float> z_in(p_in, "z");
        // Eigen::Vector3f point;
        // // Eigen::MatrixXf filtered_points
        // Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> filtered_points(3, 8000);
        // for(; x_in != x_in.end(); ++x_in, ++y_in, ++z_in){
        //     if (std::isnan(*x_in) || std::isnan(*y_in) || std::isnan(*z_in)){
        //         continue;
        //     }
        //     if (std::isinf(*x_in) || std::isinf(*y_in) || std::isinf(*z_in)){
        //         continue;
        //     }
        //     if (*x_in == 0 && *y_in == 0 && *z_in == 0){
        //         continue;
        //     }
        //     if (*z_in > 1.5 || *z_in < 0.2){
        //         continue;
        //     }
        //     point = Eigen::Vector3f(*x_in, *y_in, *z_in);
        //     // Assign point to filtered_points matrix
        //     filtered_points(0, filtered_size) = point.x();
        //     filtered_points(1, filtered_size) = point.y();
        //     filtered_points(2, filtered_size) = point.z();
        //     ROS_INFO("Point %ld: %f, %f, %f", filtered_size, point.x(), point.y(), point.z());
        //     filtered_size++;
        // }
        // // Resize the filtered_points matrix to the correct size
        // filtered_points.resize(3, filtered_size);
        // Print the size of the point cloud p_out
        // ROS_INFO("Size of the point cloud: %ld", filtered_size);
        // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        // if (filtered_size != 0) {
        //     division = int(filtered_size/5);
        //     remainder = filtered_size % 5;
        //     Eigen::MatrixXd points8 = filtered_points.block(0, 0, 3, division + remainder);
        //     Eigen::MatrixXd points1 = filtered_points.block(0, division + remainder, 3, division);
        //     Eigen::MatrixXd points2 = filtered_points.block(0, 2*division + remainder, 3, division);
        //     Eigen::MatrixXd points3 = filtered_points.block(0, 3*division + remainder, 3, division);
        //     Eigen::MatrixXd points4 = filtered_points.block(0, 4*division + remainder, 3, division);
        //     sensor_msgs::PointCloud2 transformed_pc;
        //     // Look up the transform from the world frame to the rgb frame
        //     geometry_msgs::TransformStamped transform_stamped;
        //     // Eigen::Matrix<double, 3, Eigen::Dynamic> transformed_points(3, filtered_size);
        //     try {
        //         transform_stamped = tf_buffer_->lookupTransform("hires", pc_msg->header.frame_id, ros::Time(0));
        //         // Use the transform_stamped to transform the point cloud on separate threads
        //         // by using the 4 different vectors of points
        //         translation_vector[0] = transform_stamped.transform.translation.x;
        //         translation_vector[1] = transform_stamped.transform.translation.y;
        //         translation_vector[2] = transform_stamped.transform.translation.z;
        //         tf_quat[0] = transform_stamped.transform.rotation.x;
        //         tf_quat[1] = transform_stamped.transform.rotation.y;
        //         tf_quat[2] = transform_stamped.transform.rotation.z;
        //         tf_quat[3] = transform_stamped.transform.rotation.w;
        //         q = Eigen::Quaterniond(tf_quat[3], tf_quat[0], tf_quat[1], tf_quat[2]);
        //         rotation_matrix = q.toRotationMatrix();
                // std::thread t8([&](){
                //     applyAffinity(CPUS::CPU8);
                //     transformed_points.block(0, 0, 3, points8.cols()) = rotation_matrix * points8 + translation_vector.replicate(1, points8.cols());
                // });
                // std::thread t1([&](){
                //     applyAffinity(CPUS::CPU1);
                //     transformed_points.block(0, points8.cols(), 3, points1.cols()) = rotation_matrix * points1 + translation_vector.replicate(1, points1.cols());
                // });
                // std::thread t2([&](){
                //     applyAffinity(CPUS::CPU2);
                //     transformed_points.block(0, points8.cols() + points1.cols(), 3, points2.cols()) = rotation_matrix * points2 + translation_vector.replicate(1, points2.cols());
                // });
                // std::thread t3([&](){
                //     applyAffinity(CPUS::CPU3);
                //     transformed_points.block(0, points8.cols() + points1.cols() + points2.cols(), 3, points3.cols()) = rotation_matrix * points3 + translation_vector.replicate(1, points3.cols());
                // });
                // std::thread t4([&](){
                //     applyAffinity(CPUS::CPU4);
                //     transformed_points.block(0, points8.cols() + points1.cols() + points2.cols() + points3.cols(), 3, points4.cols()) = rotation_matrix * points4 + translation_vector.replicate(1, points4.cols());
                // });
                // t8.join();
                // t1.join();
                // t2.join();
                // t3.join();
                // t4.join();
                // t5.join();
                // t6.join();
                // t7.join();
                // Create a new point cloud message to store the transformed points
        //         transformed_cloud.width = filtered_points.cols();
        //         transformed_cloud.height = 1;
        //         transformed_cloud.is_dense = false;
        //         transformed_cloud.points.resize(transformed_cloud.width * transformed_cloud.height);
        //         // std::cout << "Size of transformed points: " << transformed_points.rows() << " x " << transformed_points.cols() << std::endl;
        //         for (size_t i = 0; i < transformed_cloud.size(); ++i) {
        //             transformed_cloud.points[i].x = filtered_points(0, i);
        //             transformed_cloud.points[i].y = filtered_points(1, i);
        //             transformed_cloud.points[i].z = filtered_points(2, i);
        //         }
        //         pcl::toROSMsg(transformed_cloud, transformed_pc);
        //         transformed_pc.header.frame_id = "world";                                 

        //     } catch (tf2::TransformException &ex) {
        //         ROS_WARN("Failure, I am here %s\n", ex.what());
        //         ROS_WARN("%s", ex.what());
        //         return;
        //     }
        //     // Transform the point cloud by using transform_stamped and the point cloud message
        //     // tf2::doTransform(*pc_msg, transformed_pc, transform_stamped); // Use tf2::doTransform instead of tf2_sensor_msgs::doTransform
        //     pc_pub_.publish(transformed_pc);
        // } else {
        //     ROS_WARN("No valid points in the point cloud message");
        // }
    }

private:
    tf2_ros::Buffer* tf_buffer_;
    
    tf2_ros::TransformListener* tf_listener_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
    // Eigen::MatrixXd init_points;
    // Eigen::MatrixXd transformed_points;
    // Eigen::MatrixXd points1;
    // Eigen::MatrixXd points2;
    // Eigen::MatrixXd points3;
    // Eigen::MatrixXd points4;
    // Eigen::MatrixXd points5;
    // Eigen::MatrixXd points6;
    // Eigen::MatrixXd points7;
    // Eigen::MatrixXd points8;
    double tf_quat[4];
    Eigen::Quaterniond q;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d translation_vector;
    // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // pcl::PointCloud<pcl::PointXYZ> init_cloud;
    size_t filtered_size;
    int remainder;
    int division;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_transform");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}
