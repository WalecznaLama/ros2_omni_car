from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_file = PathJoinSubstitution([FindPackageShare("omni_car"), "config", "mapper_params_online_sync.yaml"])

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("slam_toolbox"), "/launch/online_sync_launch.py"]),
        launch_arguments={"slam_params_file": config_file}.items(),
    )

    return LaunchDescription([slam_toolbox])
