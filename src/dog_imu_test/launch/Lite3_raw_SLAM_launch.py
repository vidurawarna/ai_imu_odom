from launch import LaunchDescription
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import os
import math

remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]

launch_dir = os.path.dirname(__file__)  # directory of launch file
ws_root = os.path.abspath(os.path.join(launch_dir, '../../../../../'))
bag_path = os.path.join(ws_root, 'bag_files', 'staircase_bags_sports_complex2')

def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')
    bag_file = LaunchConfiguration('bag_file')
  
    foo_dir = get_package_share_directory('dog_imu_test')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    )

    declare_bag_file = DeclareLaunchArgument(
        'bag_file',
        default_value=bag_path
    )

    declare_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='go2'
    )

    ai_odom_pkg_dir = get_package_share_directory('ai_odom_3d_ros2')
    ai_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ai_odom_pkg_dir + '/launch/ai_odom.launch.py'
        ),
        launch_arguments={
            'namespace': robot_namespace,
        }.items()
    
    )

    camera_init_to_odom_tf_publisher =Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name = 'start_imu_to_lidar_tf_publisher',
            remappings=remappings,
            arguments = ['--x', str(0), '--y', str(0), '--z', str(0), '--yaw', str(0), '--pitch', str(0), '--roll', str(0), '--frame-id', 'camera_init', '--child-frame-id', 'odom']
        )

    pointcloud_to_laserscan_node = ExecuteProcess(
        cmd=[
                'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
                '--ros-args',
                    '-r', 'cloud_in:=cloud_registered_body',
                    '-r', 'scan:=laserscan',
                    '-r', '/tf:=tf',
                    '-r', '/tf_static:=tf_static',

                    '-p', 'target_frame:=body',
                    '-p', 'range_min:=0.0',
                    '-p', 'range_max:=50.0',
                    '-p', 'angle_min:=-3.1415',
                    '-p', 'angle_max:=3.1415',
                    '-p', 'use_inf:=true',
                    '-p', 'inf_epsilon:=0.0',
                    '-p', 'min_height:=0.1',
                    '-p', 'max_height:=1.0',
                    '-p', 'angle_increment:=0.0087',
                    '-p', 'scan_time:=0.3333',
                    '-p', 'transform_tolerance:=0.5',
                    '-p','use_sim_time:=true',
            ],
            output='screen',
        )
    

    
    #------------------------------------ SLAM toolbox launch -------------------------------
    #----------------------------------------------------------------------------------------
    start_async_slam_toolbox_node = Node(    
        parameters=[
        
            ParameterFile(os.path.join(foo_dir, 'config', 'mapper_params_online_async_lite3.yaml'), allow_substs=True),
            # ParameterFile(os.path.join(imu_bringup_pkg, 'config', 'slam_toolbox_async_fusion.yaml'), allow_substs=True),
            {   # Override with string literals
                "odom_frame": "camera_init",
                "map_frame": "map",
                "base_frame": "body",
                "scan_topic": "laserscan"
            }
        ],
        
        remappings=[('/tf_static', 'tf_static'), 
                    ('/tf', 'tf'),
                    ("/map", "map"),                 
                    ("/map_updates", "map_updates"),],

        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args','-p','use_sim_time:=true'],
        # arguments=['--use-sim-time','true'],
    )

    #------------------------------------ Visualization -------------------------------------
    #----------------------------------------------------------------------------------------
   
    rviz = Node(
            package='rviz2',
            remappings=remappings,
            executable='rviz2',
            name='lite3_rviz2',
            arguments=['-d', foo_dir + '/config/lite3_SLAM.rviz'],
            # arguments=['--ros-args','-p','use_sim_time:=true'],
        )
    
    tftree = Node(
            package='rqt_tf_tree',
            remappings=remappings,
            executable='rqt_tf_tree',
            name='tf_tree',
            arguments=['--ros-args','-p','use_sim_time:=true'],
            # arguments=['--use-sim-time','true'],
        )
    
    #----------------------------------- Bag replay ------------------------------------------
    #-----------------------------------------------------------------------------------------

    bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file ,
                '--remap',
                '/imu/data:=/vectornav/imu', 
                '--clock',
                '-r','1'],
            output='screen',
        )
    
    
    
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_bag_file)
    ld.add_action(declare_namespace)
    ld.add_action(camera_init_to_odom_tf_publisher)

    # ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(ai_odom_launch)

    # ----------- visualization nodes--------------
    ld.add_action(rviz)
    # ld.add_action(tftree)

    #--------------- SLAM--------------------------
    # ld.add_action(start_async_slam_toolbox_node)
    
    #--------------- play back bag ----------------
    ld.add_action(bag)
    

    return ld
