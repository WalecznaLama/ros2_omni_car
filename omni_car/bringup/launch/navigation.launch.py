from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_file = PathJoinSubstitution([FindPackageShare("omni_car"), "config", "nav2_params.yaml"])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("nav2_bringup"), "/launch/navigation_launch.py"]),
        launch_arguments={"params_file": config_file}.items(),
    )

    return LaunchDescription([navigation])
