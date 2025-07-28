import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    nav2_yaml = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'nav2.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('fastbot_slam'), 'rviz', 'fastbot_map.rviz')
    map_file = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'fastbot_map.yaml')
    bt_nav_yaml = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'bt_navigator.yaml')
    controller_yaml = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'controller.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'recovery.yaml')
    planner_yaml = os.path.join(get_package_share_directory('fastbot_slam'), 'config', 'planner_server.yaml')
   
    return LaunchDescription([
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} ]),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]),

        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_dir],
        #    parameters=[{'use_sim_time': True}],
        #    output='screen'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_yaml]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_nav_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator']}])
    ])
