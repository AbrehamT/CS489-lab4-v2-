from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    pkg_arg = DeclareLaunchArgument(
        'pkg',
        default_value='abreham_safety_node',
        description='Package name that contains the node to run'
    )
    exe_arg = DeclareLaunchArgument(
        'exe',
        default_value='abreham_safety_node.py',
        description='Executable name for the safety node'
    )
    mode_arg = DeclareLaunchArgument("mode", default_value="sim", description="Simulation mode")
    student_arg = DeclareLaunchArgument("student", default_value="abe", description="Student name")
    ttc_arg = DeclareLaunchArgument("ttc", default_value="2.0", description="Time-to-collision threshold")

    ld.add_action(pkg_arg)
    ld.add_action(exe_arg)
    ld.add_action(mode_arg)
    ld.add_action(student_arg)
    ld.add_action(ttc_arg)

    
    safety_node = Node(
        package=LaunchConfiguration('pkg'),
        executable=LaunchConfiguration('exe'),
        name='safety_node',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'student': LaunchConfiguration('student'),
            'ttc': LaunchConfiguration('ttc')
        }]
    )

    ld.add_action(safety_node)

    return ld
