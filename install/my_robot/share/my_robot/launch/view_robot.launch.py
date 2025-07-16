from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot')

    xacro_file = os.path.join(pkg_path, 'urdf', 'four_wheel_bot.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    world_path = os.path.join(pkg_path, 'world', 'obstacle_world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'four_wheel_bot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.25',
                '-Y', '0.0'
            ],
            output='screen'
    ),

        Node(
            package='my_robot',
            executable='stopper_node',
            name='stopper_node',
            output='screen'
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()

