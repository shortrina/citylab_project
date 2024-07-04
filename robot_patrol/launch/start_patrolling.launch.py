from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


#DeclareLaunchArgument(name='--log-level', default='debug')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='robot_patrol_node',            
            output='screen',            
            emulate_tty=True,
            ros_arguments = ['--log-level', 'rclcpp:=ERROR', '--log-level', 'debug:=INFO']            
        ),
    ])

