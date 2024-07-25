import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 
import launch_ros.actions

def generate_launch_description():

    # Define the path to the map file
    map_server_config_path = os.path.join(get_package_share_directory('omni_car'), 'maps', 'map.yaml')

    # Node to start the map server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'yaml_filename': map_server_config_path}]
    )

    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]
    )

    # Node to publish the static transform between map and odom
    static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    ld = LaunchDescription()
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(static_transform_publisher_cmd)

    return ld