#!/usr/bin/env python3
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='precision_landing')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='precision_landing',
            description='Namespace of the nodes'
        ),
        Node(
            package='precision_landing',
            namespace=namespace,
            executable='visualizer.py',
            name='visualizer',
            parameters=[
                {'namespace': namespace},
                {'use_sim_time': True} # Simülasyon saati
            ],
            # TF verilerini namespace dışına, global kanala çıkarır
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),
        OpaqueFunction(function=launch_setup),
    ])

def patch_rviz_config(original_config_path, namespace):
    with open(original_config_path, 'r') as f:
        content = f.read()

    content = content.replace('__NS__', f'/{namespace}' if namespace else '')
    
    tmp_rviz_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    tmp_rviz_config.write(content.encode('utf-8'))
    tmp_rviz_config.close()

    return tmp_rviz_config.name


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    
    pkg_dir = get_package_share_directory('precision_landing')
    rviz_config_path = os.path.join(pkg_dir, 'resources', 'visualize.rviz')
    
    patched_config = patch_rviz_config(rviz_config_path, namespace)

    return [
        # Orijinaldeki map->base_link silindi. Yerine Lidar (link) -> base_link bağı eklendi.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'link'],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', patched_config],
            parameters=[{'use_sim_time': True}]
        ),
    ]