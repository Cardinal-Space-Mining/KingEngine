from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
<<<<<<< HEAD
    current_pkg = FindPackageShare('king_engine')
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
    
=======

# sick_scan_xd
>>>>>>> main
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

    # perception_node = Node(
    #     package='sick_perception',
    #     executable = 'ldrp_node',
    #     output = 'screen',
    #     remappings=[
    #         ('/uesim/scan', '/cloud_all_fields_fullframe'),
    #         ('/uesim/pose', 'dlo/odom_node/pose'),
    #         ('/ldrp/obstacle_grid', 'perception/obstacle_grid')
    #     ]
    # )
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

    # path_plan_node = Node(
    #     package = 'path_plan',
    #     executable = 'main',
    #     output = 'screen',
    #     remappings=[
    #         ('lidar_map', 'perception/obstacle_grid'),
    #         ('location', 'dlo/odom_node/pose'),
    #         ('destination', 'king_engine/destination'),
    #         ('path', 'path_plan/path'),
    #         ('weight_map', 'path_plan/weight_map')
    #     ]
    # )
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

# rio_interface
    rio_interface_node = Node(
        package = 'rio_interface',
        executable = 'rio_interface',
        output = 'screen',
        # remapping=[
        #     ('set_track_velocity', 'rio_interface/set_track_velocity'),
        #     ('start_mining', 'rio_interface/start_mining'),
        #     ('stop_mining', 'rio_interface/stop_mining'),
        #     ('start_offload', 'rio_interface/start_offload')
        # ]
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
        rio_interface_node,
        video_publisher_node
    ])
