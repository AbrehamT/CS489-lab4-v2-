from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="sim", description="Simulation mode"
    )
    ttc_arg = DeclareLaunchArgument(
        "ttc", default_value="2.0", description="Time-to-collision threshold"
    )

    safety_node = Node(
        package='richard_safety_node',  
        executable='richard_safety_node.py', 
        name='richard_safety_node', 
        output='screen',
        parameters=[
            {'mode': LaunchConfiguration('mode')},
            {'ttc': LaunchConfiguration('ttc')}
        ]
    )

    return LaunchDescription([
        mode_arg,
        ttc_arg,
        safety_node,
    ])
