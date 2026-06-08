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
import yaml

def load_params():
    config_file = os.path.join(
        get_package_share_directory('dog_imu_test'),
        'config',
        'tf_params.yaml'
    )
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)['tf_parameters']

tf_params = load_params()

# W.R.T. imu_link
xyz_imu_lidar = tf_params['xyz_imu_lidar']  
rpy_imu_lidar = tf_params['rpy_imu_lidar']

xyz_imu_baselink = tf_params['xyz_imu_baselink']  
rpy_imu_baselink = tf_params['rpy_imu_baselink']

xyz_imu_basefootprint = tf_params['xyz_imu_basefootprint']  
rpy_imu_basefootprint = tf_params['rpy_imu_basefootprint']

# W.R.T. lidar
xyz_lidar_imu = tf_params['xyz_lidar_imu']  
rpy_lidar_imu = tf_params['rpy_lidar_imu']

xyz_lidar_baselink = tf_params['xyz_lidar_baselink']  
rpy_lidar_baselink = tf_params['rpy_lidar_baselink']

xyz_baselink_basefootprint = tf_params['xyz_baselink_basefootprint']  
rpy_baselink_basefootprint = tf_params['rpy_baselink_basefootprint']


for i in range (len(rpy_imu_lidar)):
    rpy_imu_lidar[i] = math.radians(rpy_imu_lidar[i])
    rpy_imu_baselink[i] = math.radians(rpy_imu_baselink[i])
    rpy_imu_basefootprint[i] = math.radians(rpy_imu_basefootprint[i])

    rpy_lidar_imu[i] = math.radians(rpy_lidar_imu[i])
    rpy_lidar_baselink[i] = math.radians(rpy_lidar_baselink[i])
    rpy_baselink_basefootprint[i] = math.radians(rpy_baselink_basefootprint[i])

remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]

launch_dir = os.path.dirname(__file__)  # directory of launch file
ws_root = os.path.abspath(os.path.join(launch_dir, '../../../../../'))
bag_path = os.path.join(ws_root, 'bag_files', 'CURTAIN_test2_bag_10')

def generate_launch_description():

    robot_namespace = LaunchConfiguration('robot_namespace')
    bag_file = LaunchConfiguration('bag_file')
  
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

    declare_bag_file = DeclareLaunchArgument(
        'bag_file',
        default_value=bag_path
    )
    declare_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='go2'
    )
  
    start_tf_publisher = Node(
            package='dog_imu_test',
            executable='odom_to_imu_tf_broadcaster',
            name='odom_to_imu_tf_broadcaster',
            namespace=robot_namespace,
            remappings=remappings,
            output="screen",
            arguments=['--use-sim-time','true'],
           )

    start_imu_to_lidar_tf_publisher =Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name = 'start_imu_to_lidar_tf_publisher',
            namespace=robot_namespace,
            remappings=remappings,
            arguments = ['--x', str(xyz_imu_lidar[0]), '--y', str(xyz_imu_lidar[1]), '--z', str(xyz_imu_lidar[2]), '--yaw', str(rpy_imu_lidar[2]), '--pitch', str(rpy_imu_lidar[1]), '--roll', str(rpy_imu_lidar[0]), '--frame-id', 'imu_link', '--child-frame-id', 'livox_frame']
        )
    
    start_imu_to_baselink_tf_publisher=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name = 'start_imu_to_baselink_tf_publisher',
            namespace=robot_namespace,
            remappings=remappings,
            arguments = ['--x', str(xyz_imu_baselink[0]), '--y', str(xyz_imu_baselink[1]), '--z', str(xyz_imu_baselink[2]), '--yaw', str(rpy_imu_baselink[2]), '--pitch', str(rpy_imu_baselink[1]), '--roll', str(rpy_imu_baselink[0]), '--frame-id', 'imu_link', '--child-frame-id', 'base_link']
        )
    
    start_imu_to_basefootprint_tf_publisher=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name = 'start_imu_to_basefootprint_tf_publisher',
            namespace=robot_namespace,
            remappings=remappings,
            arguments = ['--x', str(xyz_imu_basefootprint[0]), '--y', str(xyz_imu_basefootprint[1]), '--z', str(xyz_imu_basefootprint[2]), '--yaw', str(rpy_imu_basefootprint[2]), '--pitch', str(rpy_imu_basefootprint[1]), '--roll', str(rpy_imu_basefootprint[0]), '--frame-id', 'imu_link', '--child-frame-id', 'base_footprint']
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
            arguments=['-d', this_pkg_dir + '/config/test_config.rviz'],
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
                '--topics',
                '/vectornav/imu',
                '/go2/livox/lidar',
                '/go2/map',
                '/go2/laserscan',
                '--remap',
                '/go2/map:=/LIO_map',
                '/go2/laserscan:=/scan',
                '--clock'
                ],
            output='screen',
        )
    
    start_async_slam_toolbox_node = Node(
        
        parameters=[
        
            ParameterFile(os.path.join(this_pkg_dir, 'config', 'mapper_params_online_async.yaml'), allow_substs=True)
        ],
        namespace=robot_namespace,
        remappings=remappings,
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # pointcloud_to_laserscan = ExecuteProcess(
    #     cmd = ['ros2','run','pointcloud_to_laserscan','pointcloud_to_laserscan_node',
    #            '--ros-args',
    #            '-p','target_frame:=base_footprint',
    #            '-p','qos_overrides./cloud_in.reliability:=reliable',
    #            '-p', 'range_min:=0.0', 
    #             '-p', 'range_max:=30.0', 
    #             '-p', 'angle_min:=-3.1415', 
    #             '-p', 'angle_max:=3.1415', 
    #             '-p','use_inf:=true',
    #             '-p','inf_epsilon:=0.0',
    #             '-p','min_height:=0.0',
    #             '-p','max_height:=1.0',
    #             '-p','angle_increment:=0.0087',
    #             '-p','scan_time:=0.3333',
    #             '-p','transform_tolerance:=0.5',
    #            '--remap','__ns:=/go2',
    #            '--remap','cloud_in:=/go2/livox/lidar',
    #            '--remap','scan:=/go2/scan',
    #            '--remap','/tf:=/go2/tf',
    #            '--remap','/tf_static:=/go2/tf_static',
    #            ],
    #     output = 'screen',
    # )

    ld = LaunchDescription()

    ld.add_action(declare_bag_file)
    ld.add_action(declare_namespace)
    
    ld.add_action(start_imu_to_lidar_tf_publisher)
    ld.add_action(start_imu_to_baselink_tf_publisher)
    ld.add_action(start_imu_to_basefootprint_tf_publisher)
    ld.add_action(map_to_odom_init)

    ld.add_action(bag)
    ld.add_action(start_tf_publisher)
    # ld.add_action(pointcloud_to_laserscan)

    ld.add_action(rviz)
    # ld.add_action(tftree)
    ld.add_action(ai_odom_launch)
    ld.add_action(start_async_slam_toolbox_node)
    return ld