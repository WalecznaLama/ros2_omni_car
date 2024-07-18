from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rqt",
            default_value="false",
            description="Start rqt automatically with this launch file.",
        )
    )

    # Initialize Arguments
    arg_rviz = LaunchConfiguration("rviz")
    arg_rqt = LaunchConfiguration("rqt")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("omni_car"), "urdf", "omni_car.xacro"]), " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    world_sdf_file = PathJoinSubstitution([FindPackageShare("omni_car"), "gazebo", "world.sdf"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("omni_car"), "rviz", "rviz.rviz"])

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),  # -v verbose level, -r run immediately 
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "omni_car",
            "-allow_renaming", "true",
            "-z", "0.06"
        ],
    )
    gz_spawn_world = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", world_sdf_file,
            "-name", "world",
            "-allow_renaming", "true",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'
                   ],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(arg_rviz),
    )

    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/camera'],
        condition=IfCondition(arg_rqt),
    )

    nodes = [
        gazebo,
        gz_spawn_entity,
        gz_spawn_world,
        joint_state_publisher,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        bridge,
        rviz,
        rqt
    ]

    return LaunchDescription(declared_arguments + nodes)
