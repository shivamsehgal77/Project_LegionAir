#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs import do_transform_cloud

class PointCloudTransformer_RGB:
    def __init__(self):
        rospy.init_node("point_cloud_transformer_rgb", anonymous=True)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create publishers for transformed point clouds
        self.rgb_pcl_pub = rospy.Publisher("/rgb_pcl", PointCloud2, queue_size=15)

        # Subscribe to the /tof_pc topic
        rospy.Subscriber("/tof_pc", PointCloud2, self.point_cloud_callback_rgb)

    def point_cloud_callback_rgb(self, tof_pcl_msg):
        try:
            # Transform point cloud from world to body frame
            rgb_pcl_msg = self.transform_point_cloud_rgb(tof_pcl_msg, "world", "hires")
            self.rgb_pcl_pub.publish(rgb_pcl_msg)

            # Perform sanity check
            # self.transform_known_point()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed, skipping point cloud transformation.")

    def transform_point_cloud_rgb(self, input_pcl, source_frame, target_frame):
        try:
            # Attempt to get transform from source_frame to target_frame
            
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

            # Transform the input point cloud
            transformed_pcl_msg = do_transform_cloud(input_pcl, transform_stamped)

            return transformed_pcl_msg

        except tf2_ros.LookupException:
            rospy.logwarn("Transform lookup failed, skipping point cloud transformation.")
            return None

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        pcl_transformer_rgb = PointCloudTransformer_RGB()
        pcl_transformer_rgb.run()
    except rospy.ROSInterruptException:
        pass
