from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            remappings=[
                ('/cmd_vel', '/omni_car_controller/cmd_vel')
            ],
            parameters=[{
                'require_enable_button': False,
                'enable_button': 0,
                'axis_linear.x': 1,
                'axis_linear.y': 0,
                'scale_linear.x': 0.5,
                'scale_linear.y': 0.5,
                'axis_angular.z': 2,
                'scale_angular': 1.5,
            }]
        )
    ])
