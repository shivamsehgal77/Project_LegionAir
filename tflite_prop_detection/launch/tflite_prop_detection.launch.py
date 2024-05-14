import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    tflite_node_params = PathJoinSubstitution(
        [FindPackageShare("tflite_prop_detection"), "config", "ns_conf.yaml"]
    )
    pack_path = os.path.join(get_package_share_path('tflite_prop_detection'), "config", "ns_conf.yaml")
    print(pack_path)
    with open(pack_path) as file:
        config = yaml.safe_load(file)

    # Extract the id parameter
    id_value = config['tflite_prop_detection']['ros__parameters']['id']
    ld = LaunchDescription()
    
    tf_static_node = Node(
        package="your_tf_package",
        executable="tf_publisher_cpp",
        output="screen",
    )

    pc_transform_node = Node(
        package="your_pointcloud_package", 
        executable="pointcloud_transformer", 
        output="screen",
        parameters=[tflite_node_params]
    )

    obj_det_node = Node(
        package="tflite_prop_detection", 
        executable="tflite_prop_detection_cpp", 
        output="screen",
        parameters=[tflite_node_params]
    )
    tf_static_node_with_namespace = GroupAction( 
        actions = [PushRosNamespace('uav_' + str(id_value)), tf_static_node]
    )
    pc_transform_node_with_namespace = GroupAction( 
        actions = [PushRosNamespace('uav_' + str(id_value)), pc_transform_node]
    )
    obj_det_node_with_namespace = GroupAction( 
        actions = [PushRosNamespace('uav_' + str(id_value)), obj_det_node]
    )
    
    ld.add_action(tf_static_node_with_namespace)
    ld.add_action(pc_transform_node_with_namespace)
    ld.add_action(obj_det_node_with_namespace)
    return ld
