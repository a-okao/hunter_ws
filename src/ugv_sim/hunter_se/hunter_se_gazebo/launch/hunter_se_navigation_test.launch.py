import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    hunter_se_dir = get_package_share_directory('hunter_se_gazebo')
    hunter_se_description_dir = get_package_share_directory('hunter_se_description')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', 
                                      default=os.path.join(hunter_se_dir, 'maps', 'factory_map.yaml'))
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    world = LaunchConfiguration('world', default='factory')

    # World file selection
    factory_world_path = os.path.join(hunter_se_dir, 'world', 'factory.world')

    # Gazebo launch
    gazebo_options_dict = {
        'world': factory_world_path,
        'verbose': 'true'
    }

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_options_dict.items()
    )

    # Robot spawn options
    car_sim_options = {
        'start_x': '0',
        'start_y': '0',
        'start_z': '0.1',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    # Spawn Hunter SE robot
    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(hunter_se_dir, 'launch', 'hunter_se_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    # Nav2 launch with localization (no SLAM)
    nav2_options = {
        'use_sim_time': 'true',
        'slam': 'False',  # Disable SLAM for navigation
        'map': map_yaml_file,
        'params_file': os.path.join(hunter_se_dir, 'config', 'nav2', 'nav2_params.yaml'),
        'autostart': 'true',
        'use_composition': 'True',
        'use_respawn': 'False'
    }

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments=nav2_options.items()
    )

    # RViz2 configuration for navigation
    rviz_config_path = os.path.join(
        hunter_se_description_dir,
        'rviz',
        'hunter_se_nav2.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(hunter_se_dir, 'maps', 'factory_map.yaml'),
            description='Full path to the map yaml file'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 if true'),
        DeclareLaunchArgument(
            'world',
            default_value='factory',
            description='World to use (factory or house)'),
        
        gazebo_simulator,
        spawn_car,
        nav2_bringup,
        rviz_node,
    ])