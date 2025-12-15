#!/usr/bin/env python3
"""
Launch file per IK-based Pick & Place demo
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Argomento per scegliere se lanciare anche Gazebo
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Se true, lancia anche Gazebo; se false, assume Gazebo già running'
    )
    
    use_gazebo = LaunchConfiguration('use_gazebo')
    
    # Include Gazebo world (opzionale)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('fra2mo_armando'),
                'launch',
                'launch_world.launch.py'
            )
        ]),
        condition=lambda context: context.perform_substitution(use_gazebo) == 'true'
    )
    
    # Nodo task Pick & Place
    pick_place_node = Node(
        package='fra2mo_armando',
        executable='ik_pick_place.py',
        name='ik_pick_place_task',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        use_gazebo_arg,
        # gazebo_launch,  # Commentato: lancia Gazebo separatamente
        pick_place_node
    ])
