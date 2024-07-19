from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_to_twist_stamped',
            executable='twist_to_twist_stamped',
            name='twist_to_twist_stamped',
            output='screen',
            remappings=[('/cmd_vel_in', '/omni_drive_controller/cmd_vel_twist'),
                        ('/cmd_vel_out', '/omni_drive_controller/cmd_vel')],
            parameters=[ {'frame_id': 'base_link'} ]

        )
    ])
