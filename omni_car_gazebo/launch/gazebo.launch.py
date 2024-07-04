import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Paths to URDF and world files
    package_name_model = 'omni_car_description'
    package_name = 'omni_car_gazebo'
    urdf_file_name = 'urdf/omni_car.xacro'
    world_file_name = 'worlds/empty.world'

    urdf = os.path.join(get_package_share_directory(package_name_model), urdf_file_name)
    world = os.path.join(get_package_share_directory(package_name), world_file_name)

    # Process the Xacro file to generate the robot description in XML format
    robot_description = xacro.process_file(urdf).toxml()

    return LaunchDescription([

        # Declare the use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Start Gazebo server with the specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world , '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Node to publish the robot state to the ROS 2 ecosystem
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
        ),

        # Node to publish the joint states of the robot
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Node to spawn the robot model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "omni_car", '-x', '0', '-y', '0', '-z', '0.5']
        ),
    ])
