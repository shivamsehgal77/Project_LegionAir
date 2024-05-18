# Import necessary libraries
from launch import LaunchDescription
import os
import yaml
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_path

# Define the launch description
def generate_launch_description():
    ld = LaunchDescription()
    # Declare a command-line argument "cmd_line_parameter"
    # cmd_line_parameter = DeclareLaunchArgument(
    #     "mode",
    #     default_value="away",
    #     description="A parameter from the command line.",
    # )
    # Path to the parameters file
    node_params = PathJoinSubstitution(
        [FindPackageShare("drone_cpp"), "config", "params.yaml"]
    )
    pack_path = os.path.join(get_package_share_path("drone_cpp"), "config", "params.yaml")
    print(pack_path)
    with open(pack_path) as file:
        config = yaml.safe_load(file)
    id_value = config['drone_node_cpp']['ros__parameters']['id']
    # node_params_1 = PathJoinSubstitution(
    #     [FindPackageShare("drone_cpp"), "config", "params_1.yaml"]
    # )
    # node_params_2 = PathJoinSubstitution(
    #     [FindPackageShare("drone_cpp"), "config", "params_2.yaml"]
    # )
    # node_params_3 = PathJoinSubstitution(
    #     [FindPackageShare("drone_cpp"), "config", "params_3.yaml"]
    # )
    
    drone_node_cpp=Node(
            package='drone_cpp',
            name='drone_'+ str(id_value),
            executable='drone_node_cpp',
            parameters=[node_params],
            
    )
    print("Drone zero is created")
    
    # drone_node_cpp_1=Node(
    #         package='drone_cpp',
    #         name='drone_one',
    #         executable='drone_node_cpp',
    #         parameters=[node_params_1],
            
    # )
    # print("Drone one is created")
    # drone_node_cpp_2=Node(
    #         package='drone_cpp',
    #         name='drone_two',
    #         executable='drone_node_cpp',
    #         parameters=[node_params_2],
            
    # )
    # print("Drone two is created")
    # drone_node_cpp_3=Node(
    #         package='drone_cpp',
    #         name='drone_three',
    #         executable='drone_node_cpp',
    #         parameters=[node_params_3],
            
    # )
    # print("Drone three is created")
    
    # Add the actions to the launch description
    # ld.add_action(cmd_line_parameter)
    ld.add_action(drone_node_cpp)
    # ld.add_action(drone_node_cpp_1)
    # ld.add_action(drone_node_cpp_2)
    # ld.add_action(drone_node_cpp_3)
  
    return ld