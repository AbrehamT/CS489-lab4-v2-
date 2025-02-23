#!/usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    wall_follow_node = launch_ros.actions.Node(
        package='oscar_wall_follow',
        executable='oscar_wall_follow_node.py',  # Script name installed into lib/oscar_wall_follow
        name='wall_follow_node',
        output='screen',
        parameters=[
            {"speed": 2.0},
            {"P": 1.0},
            {"I": 0.0},
            {"D": 1.0},
        ]
    )

    return launch.LaunchDescription([wall_follow_node])
