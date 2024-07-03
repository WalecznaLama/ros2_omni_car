import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directories
    display_file_dir = get_package_share_directory('omni_car')
    # control_dir = get_package_share_directory('omni_car_controller')
    gazebo_dir = get_package_share_directory('omni_car_gazebo')

    # Create the launch configurations
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')
        )
    )

    # control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(control_dir, 'launch', 'control.launch.py')
    #     )
    # )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(display_file_dir, 'launch', 'display.launch.py')
        ),
        launch_arguments={'publish_robot_state': 'false'}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        # control_launch,
        display_launch,
    ])
