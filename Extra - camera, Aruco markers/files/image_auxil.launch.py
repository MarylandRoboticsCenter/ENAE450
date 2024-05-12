#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='image_transport',
            executable='republish',
            arguments=['compressed', 'raw'],
            name='transport_node',
            namespace='camera/image_raw',
            output='screen',
            remappings=[
                ('in/compressed', 'compressed'),
                ('out', 'image_raw')
            ],
        ),

        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            namespace='camera/image_raw',
            output='screen',
            remappings=[
                ('image', 'image_raw'),
                ('image_rect', 'image_rect')
            ],
        ),
    ])
