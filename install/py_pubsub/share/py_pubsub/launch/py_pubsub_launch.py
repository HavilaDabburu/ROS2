from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            executable='number_publisher',
            name='number_publisher',
            output='screen'
        ),
        Node(
            package='py_pubsub',
            executable='square_subscriber',
            name='square_subscriber',
            output='screen'
        ),
    ])
