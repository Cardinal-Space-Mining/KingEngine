from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')
    dlio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('direct_lidar_inertial_odometry'),
                    'launch',
                    'dlio.launch.py'
                ])
            ]),
            launch_arguments={
                'rvis': 'true',
                'pointcloud_topic': '/cloud_all_fields_fullframe',
                'imu_topic': '/sick_scansegment_xd/imu'
            }.items()
        )
    
    sick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sick_scan_xd'),
                'launch',
                'sick_multiscan.launch.py'
            ])
        ]),
        launch_arguments={
            'hostname': '10.11.11.3',
            'udp_receiver_ip': '{hostIP}',
        }.items()
    )

    path_plan_node = Node(
        package = 'path_plan',
        executable = 'main',
        output = 'screen',
        remappings=[
            ('lidar_map', 'perception/obstacle_grid'),
            ('location', 'dlio/odom_node/pose'),
            ('destination', 'king_engine/destination'),
            ('path', 'path_plan/path'),
            ('weight_map', 'path_plan/weight_map')
        ]
    )

    perception_node = Node(
        package='sick_perception',
        executable = 'ldrp_node',
        output = 'screen',
        remappings=[
            ('/uesim/scan', '/cloud_all_fields_fullframe'),
            ('/uesim/pose', 'dlio/odom_node/pose'),
            ('/ldrp/obstacle_grid', 'perception/obstacle_grid')
        ]
    )

    king_engine_node = Node(
        package = 'king_engine',
        executable = 'main',
        output = 'screen',
        remappings=[
            ('location', 'dlio/odom_node/pose'),
            ('destination', 'king_engine/destination')
        ]
    )

    traversal_node = Node(
        package = 'traversal',
        executable = 'main',
        output = 'screen',
        remappings=[
            ('location', 'dlio/odom_node/pose')
            ('path', 'path_planning/path')
        ]
    )

    return LaunchDescription([
        dlio_launch,
        sick_launch,
        path_plan_node,
        perception_node,
        king_engine_node,
        traversal_node
    ])




