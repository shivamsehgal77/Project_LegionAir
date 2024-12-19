import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path



def generate_launch_description():

    offboard_node_params = PathJoinSubstitution(
        [FindPackageShare('px4_ros_com'), "config", "ns_conf.yaml"]
    )
    pack_path = os.path.join(get_package_share_path('px4_ros_com'), "config", "ns_conf.yaml")
    print(pack_path)
    with open(pack_path) as file:
        config = yaml.safe_load(file)

    launch_tflite_package = os.path.join(get_package_share_path('tflite_prop_detection'),"launch","tflite_prop_detection.launch.py")

    include_tflite_package = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_tflite_package))


    id_value = config['offboard_control_node']['ros__parameters']['id']
    ld = LaunchDescription()
    

    offboard_control_node = Node(
        package="px4_ros_com", 
        executable="offboard_control_constant_velocity", 
        output="screen",
        namespace='/uav_'+str(id_value),
        parameters=[offboard_node_params]
    )

    ld.add_action(TimerAction(
        period=2.0,
        actions=[offboard_control_node]
    ))

    ld.add_action(include_tflite_package)

    return ld
