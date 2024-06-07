import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
            get_package_share_directory('my_robot_bringup'),
            'config',
            'params.yaml'
    )

    my_node = Node(
        package="my_robot_controller",
        executable="params",
        name = "params",
        namespace="nsl",
        parameters= [config]  
    )
    
    ld.add_action(my_node)
    return ld