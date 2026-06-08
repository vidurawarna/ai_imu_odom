from launch import LaunchDescription
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]

launch_dir = os.path.dirname(__file__)  # directory of launch file
ws_root = os.path.abspath(os.path.join(launch_dir, '../../../../../'))
bag_path = os.path.join(ws_root, 'bag_files', 'BLIND_TEST_bag_12')

def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')
    bag_file = LaunchConfiguration('bag_file')
    ekf_mode = LaunchConfiguration('ekf_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True'
    )

    declare_bag_file = DeclareLaunchArgument(
        'bag_file',
        default_value=bag_path
    )

    declare_ekf_mode = DeclareLaunchArgument(
        'ekf_mode',
        default_value='sfekf'
    )

    declare_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='go2'
    )

    ai_odom_pkg_dir = get_package_share_directory('ai_odom_3d_ros2')
    this_pkg_dir = get_package_share_directory('dog_imu_test')
    ai_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ai_odom_pkg_dir + '/launch/ai_odom.launch.py'
        ),
        launch_arguments={
            'namespace': robot_namespace,
        }.items()
    
    )
    
    start_odom_flipper = Node(
            package='dog_imu_test',
            executable='flip_odom',
            name='flip_odom',
            namespace=robot_namespace,
            remappings=remappings,
            output="screen",
            arguments=['--use-sim-time','true'],
           )
    
    lidar_odom_repub_node = Node(
            package='dog_imu_test',
            executable='lidar_odom_repub',
            name='lidar_odom_repub',
            remappings=remappings,
            namespace=robot_namespace,
            output="screen",
            arguments=['--use-sim-time','true'],
           )

    map_to_odom_init=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name = 'odom_to_map_init',
        namespace=robot_namespace,
        remappings=remappings,
        arguments = ['--x', str(0.0), '--y', str(0.0), '--z', str(0.0), '--yaw', str(0.0), '--pitch', str(0.0), '--roll', str(0.0), '--frame-id', 'map', '--child-frame-id', 'odom']
    )
   
    rviz = Node(
            package='rviz2',
            namespace=robot_namespace,
            remappings=remappings,
            executable='rviz2',
            name='go2_rviz2',
            arguments=['-d', this_pkg_dir + '/config/odom_fusion_config.rviz'],
        )
    
    tftree = Node(
            package='rqt_tf_tree',
            namespace=robot_namespace,
            remappings=remappings,
            executable='rqt_tf_tree',
            name='go2_tf_tree',
        )

    bag = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', 
                bag_file , 
                '--topics',''
                '/vectornav/imu',
                '/go2/livox/lidar',
                '/go2/map',
                '/go2/laserscan',
                '/Odometry',
                '--remap',
                ['/go2/livox/lidar:=/', robot_namespace, '/livox/lidar'],
                ['/go2/map:=/', robot_namespace, '/LIO_map'],
                ['/go2/laserscan:=/', 'scan'],
                ['/Odometry:=/', robot_namespace, '/Odometry'],
                '--clock',
                '-r','1'
                ],
            output='screen',
        )
    
    start_async_slam_toolbox_node = Node(
        
        parameters=[
        
            ParameterFile(os.path.join(this_pkg_dir, 'config', 'mapper_params_online_async.yaml'), allow_substs=True),
            # ParameterFile(os.path.join(this_pkg_dir, 'config', 'slam_toolbox_async.yaml'), allow_substs=True)
        ],
        namespace=robot_namespace,
        remappings=remappings,
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )

    odom_fusion_ekf_node =  Node(
            namespace=robot_namespace,
            remappings=remappings,
            parameters=[ParameterFile(os.path.join(this_pkg_dir, 'config', 'ekf.yaml'), allow_substs=True)],
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            arguments=['--use-sim-time','true'],
            
        )
    
    ekf_node = Node(
            package='dog_imu_test',
            executable='ekf_node',
            condition=IfCondition(
                PythonExpression(["'", ekf_mode, "' == 'sfekf'"])
            ),
            name='ekf_node',
            namespace=robot_namespace,
            remappings=remappings,
            output="screen",
            parameters=[{'use_sim_time': True}],
           )
    ekf_node_error = Node(
            package='dog_imu_test',
            executable='ekf_pos_error_node',
            condition=IfCondition(
                PythonExpression(["'", ekf_mode, "' == 'ekf_error'"])
            ),
            name='ekf_pos_error_node',
            namespace=robot_namespace,
            remappings=remappings,
            output="screen",
            parameters=[{'use_sim_time': True}],
           )
    
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan_node',
    #     namespace=robot_namespace,
    #     remappings=remappings,
    #     parameters=[{
    #                     'topic_in_cloud': 'pointcloud_registered',
    #                     'topic_out_scan': 'scan',
    #                     # 'target_frame':  '$(env ROBOT_NAMESPACE)/base_link', #'base_footprint'  # $(var namespace) will not work here because var only works with LaunchConfiguration
    #                     'target_frame': 'base_footprint',  # $(var namespace) will not work here because var only works with LaunchConfiguration
    #                     'transform_tolerance': 0.5,
    #                     'min_height': 0.1,
    #                     'max_height': 1.0,
    #                     'angle_min': -3.1415,  # -M_PI
    #                     'angle_max': 3.1415,  # M_PI
    #                     'angle_increment': 0.0087,  # M_PI/360.0
    #                     'scan_time': 0.3333,
    #                     'range_min': 0.0,
    #                     'range_max': 30.0,
    #                     'use_inf': True,
    #                     'inf_epsilon': 0.0 #1.0,
    #                 }],                   
    #     output='screen'
    # )
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_bag_file)
    ld.add_action(declare_ekf_mode)
    ld.add_action(declare_namespace)

    ld.add_action(map_to_odom_init)
    ld.add_action(start_odom_flipper)
    ld.add_action(lidar_odom_repub_node)
    # ld.add_action(pointcloud_to_laserscan)
    ld.add_action(rviz)
    # ld.add_action(tftree)
    ld.add_action(ai_odom_launch)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(ekf_node)
    ld.add_action(ekf_node_error)
    ld.add_action(bag)

    return ld
