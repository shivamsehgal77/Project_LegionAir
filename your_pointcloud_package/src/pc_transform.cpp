#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class PointCloudTransformer {
public:
    PointCloudTransformer() {
        ros::NodeHandle nh;
        tf_buffer_ = new tf2_ros::Buffer();
        tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
        pc_sub_ = nh.subscribe("/tof_pc", 1, &PointCloudTransformer::pc_callback, this);
        pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgb_pcl", 1);
    }

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
        sensor_msgs::PointCloud2 transformed_pc;
        // Look up the transform from the world frame to the rgb frame
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform("hires", pc_msg->header.frame_id, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        // Transform the point cloud by using transform_stamped and the point cloud message
        tf2::doTransform(*pc_msg, transformed_pc, transform_stamped); // Use tf2::doTransform instead of tf2_sensor_msgs::doTransform
        pc_pub_.publish(transformed_pc);        
    }

private:
    tf2_ros::Buffer* tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    ros::Subscriber pc_sub_;
    ros::Publisher pc_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_transform");
    PointCloudTransformer pc_transformer;
    ros::spin();
    return 0;
}
