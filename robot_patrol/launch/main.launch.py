from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    service_server_launch = Node(
            package='robot_patrol',
            executable='service_server_node',
            output='screen'
        )        

    patrol_with_service_launch = Node(
            package='robot_patrol',
            executable='patrol_with_service_node',
            output='screen'
        )
    
    return LaunchDescription([
        service_server_launch,
        patrol_with_service_launch,
    ])
