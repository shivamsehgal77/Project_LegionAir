#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation


def publish_static_transforms():
    rospy.init_node("static_tf_publisher", anonymous=True)
    static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Configuration from the provided file
    extrinsics = [
        {
            "parent": "body",
            "child": "world",
            "T_child_wrt_parent": [0.068, 0.0116, 0.0168],
            "RPY_parent_to_child": [0, 270, 0],
        },
        {
            "parent": "body",
            "child": "hires",
            "T_child_wrt_parent": [0.068, -0.012, 0.015],
            "RPY_parent_to_child": [90, 270, 0],
        },
    ]

    transforms = []

    for extrinsic in extrinsics:
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = extrinsic["parent"]
        transform_stamped.child_frame_id = extrinsic["child"]

        # Keep the translation as is
        transform_stamped.transform.translation.x = extrinsic["T_child_wrt_parent"][0]
        transform_stamped.transform.translation.y = extrinsic["T_child_wrt_parent"][1]
        transform_stamped.transform.translation.z = extrinsic["T_child_wrt_parent"][2]

        # Negate the rotation (quaternion)
        rot = Rotation.from_euler("xyz", extrinsic["RPY_parent_to_child"], degrees=True)
        quaternion = rot.inv().as_quat()
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        transforms.append(transform_stamped)

    # Publish the static transforms
    static_tf_broadcaster.sendTransform(transforms)

    rospy.spin()


if __name__ == "__main__":
    try:
        publish_static_transforms()
    except rospy.ROSInterruptException:
        pass
