from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import usb_diff


def generate_launch_description():
    current_pkg = FindPackageShare('king_engine')

    
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

# direct_lidar_odometry
    dlo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('direct_lidar_odometry'),
                    'launch',
                    'dlo.launch.py'
                ])
            ]),
            launch_arguments={
                'rvis': 'true',
                'pointcloud_topic': '/cloud_all_fields_fullframe',
                'imu_topic': '/sick_scansegment_xd/imu'
            }.items()
        )

# sick_perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sick_perception'),
                'launch',
                'ldrp.launch.py'
            ])
        ]),
        launch_arguments={
            'pointcloud_topic' : '/cloud_all_fields_fullframe',
            'pose_topic' : '/dlo/odom_node/pose'
        }.items()
    )

# path_planning
    path_plan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_plan'),
                'launch',
                'nav.launch.py'
            ])
        ]),
        launch_arguments={
            'obstacles_topic' : '/ldrp/obstacle_grid',
            'pose_topic' : '/dlo/odom_node/pose',
            'destination_topic' : 'king_engine/destination'
        }.items()
    )

# king_engine
    king_engine_node = Node(
        package = 'king_engine',
        executable = 'main',
        output = 'screen',
        remappings=[
            ('location', '/dlo/odom_node/pose'),
            ('destination', '/king_engine/destination')
        ]
    )

# traversal
    traversal_node = Node(
        package = 'traversal',
        executable = 'main',
        output = 'screen',
        remappings=[
            ('location', '/dlo/odom_node/pose'),
            ('path', '/path_plan/nav_path')
        ]
    )

    video_publisher_node = Node(
        package = 'video_publisher',
        executable = 'vpub',
        output = 'screen'
    )

    return LaunchDescription([
        sick_launch,
        dlo_launch,
        perception_launch,
        path_plan_launch,
        king_engine_node,
        traversal_node,
        video_publisher_node
    ])
