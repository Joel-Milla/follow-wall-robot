from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_wall',
            executable='action_record_odom_node',
            output='screen'
        ),
        Node(
            package='follow_wall',
            executable='service_position_robot_node',
            output='screen'
        ),
        Node(
            package='follow_wall',
            executable='client_follow_wall_node',
            output='screen'
        )
    ])