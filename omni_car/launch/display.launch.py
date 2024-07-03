from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share_model = FindPackageShare(package='omni_car_description').find('omni_car_description')
    pkg_share = FindPackageShare(package='omni_car').find('omni_car')

    default_model_path = PathJoinSubstitution([pkg_share_model, 'urdf/omni_car.xacro'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz/rviz.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='publish_robot_state',
            default_value='true',
            description='Flag to enable robot_state_publisher'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }],
            condition=IfCondition(LaunchConfiguration('publish_robot_state'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
