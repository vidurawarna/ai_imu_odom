from launch import LaunchDescription
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("ai_odom_3d_ros2")

    # Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='go2',
        description='Host Name / Namespace')

    # Create Launch configuratios
    namespace = LaunchConfiguration('namespace')

    remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]
    
    # Create launch actions
    start_exploration_node = Node(
            package='ai_odom_3d_ros2',
            executable='tracking',
            name='ai_odometry',
            namespace=namespace,
            remappings=remappings,
            output="screen",
            parameters=[
            ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            arguments=['--use-sim-time','true'],
            emulate_tty=True)
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(start_exploration_node)
    
    return ld
