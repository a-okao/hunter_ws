import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_world_path = os.path.join(
        get_package_share_directory('hunter_se_gazebo'), 'world', 'house.world')

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

    car_sim_options = {
        'start_x': '0',
        'start_y': '0',
        'start_z': '0',
        'start_yaw': '0',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                         get_package_share_directory('hunter_se_gazebo'),
                         'launch', 'hunter_se_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    # RViz2 configuration file path
    rviz_config_path = os.path.join(
        get_package_share_directory('hunter_se_description'),
        'rviz',
        'hunter_se_mid360.rviz'
    )

    # Launch RViz2 with configuration
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
        gazebo_simulator,
        spawn_car,
        rviz_node,
    ])