from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    speed_arg = DeclareLaunchArgument("speed", default_value="2.0", description="Speed of the robot")
    p_arg = DeclareLaunchArgument("P", default_value="2.0", description="Proportional gain")
    i_arg = DeclareLaunchArgument("I", default_value="2.0", description="Integral gain")
    d_arg = DeclareLaunchArgument("D", default_value="2.0", description="Derivative gain")

    # Define the node and pass parameters
    wall_follow_node = Node(
        package="richard_wall_follow",
        executable="richard_wall_follow_node.py",
        name="wall_follow",
        output="screen",
        parameters=[{
            "speed": LaunchConfiguration("speed"),
            "P": LaunchConfiguration("P"),
            "I": LaunchConfiguration("I"),
            "D": LaunchConfiguration("D"),
        }]
    )

    return LaunchDescription([
        speed_arg, p_arg, i_arg, d_arg,
        wall_follow_node
    ])
