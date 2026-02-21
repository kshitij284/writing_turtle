from launch import LaunchDescription
from launch.action import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='HI',
            description='Initials for turtle to write'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim_writer',
            executable='turtle_writer_node',
            name='turtle_writer',
            parameters=[{
                'name': LaunchConfiguration('name')
            }]
        )
])
