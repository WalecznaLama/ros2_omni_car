from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    config_file = PathJoinSubstitution([FindPackageShare("omni_car"), "config", "teleop_twist_joy.yaml"])

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_file]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            remappings=[('/cmd_vel', '/omni_car_controller/cmd_vel')],
            parameters=[config_file]
        )
    ])
