import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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
                                      default=os.path.join(hunter_se_dir, 'maps', 'your_map_name.yaml'))
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Get package directory
    pkg_hunter_se_gazebo = get_package_share_directory('hunter_se_gazebo')
    
    # Custom models world file path
    gazebo_world_path = os.path.join(pkg_hunter_se_gazebo, 'world', 'custom_models.world')
    
    # Set GAZEBO_MODEL_PATH to include our models directory
    gazebo_model_path = os.path.join(pkg_hunter_se_gazebo, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # Gazebo launch with custom models world
    gazebo_options_dict = {
        'world': gazebo_world_path,
        'verbose': 'true'
    }

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_options_dict.items()
    )

    # Robot spawn options - positioned at x=-2, y=0, z=0.3
    car_sim_options = {
        'start_x': '-2',
        'start_y': '0',
        'start_z': '0.3',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    # Spawn Hunter SE robot with MID-360 sensor
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
            default_value=os.path.join(hunter_se_dir, 'maps', 'your_map_name.yaml'),
            description='Full path to the map yaml file'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 if true'),
        DeclareLaunchArgument(
            'start_x',
            default_value='-2',
            description='Robot start X position'),
        DeclareLaunchArgument(
            'start_y',
            default_value='0',
            description='Robot start Y position'),
        DeclareLaunchArgument(
            'start_z',
            default_value='0.3',
            description='Robot start Z position'),
        DeclareLaunchArgument(
            'start_yaw',
            default_value='0',
            description='Robot start yaw orientation'),
        
        set_gazebo_model_path,
        gazebo_simulator,
        spawn_car,
        nav2_bringup,
        rviz_node,
    ])