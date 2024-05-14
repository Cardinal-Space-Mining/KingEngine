from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import subprocess, re

def get_video_streams() -> tuple[str]:
    files = subprocess.run(("ls", "/dev"), capture_output=True).stdout.decode().split('\n')
    pattern = re.compile('video\d+')
    return tuple(f"/dev/{x}" for x in files if pattern.match(x))

def get_MJPG_video_stream_from_serial_number(num: str) -> None | str:
    video_streams: tuple[str] = get_video_streams()
    results: tuple[str] = tuple(subprocess.run(("v4l2-ctl", f"--device={stream}", "--all"), capture_output=True).stdout.decode() for stream in video_streams)
    for idx, res in enumerate(results):
        if num in res and 'MJPG' in res:
            return video_streams[idx]
    raise RuntimeError(f"Video stream for serial number {num} not found")
        
# serial_numbers = ("YLAF20221208V2","CSM15424", "YLAF20221208V1")
try:
    right_cam_sn = "YLAF20221208V2"
    left_cam_sn = "YLAF20221208V1"
    center_cam_sn = "CSM15424"

    right_cam_stream = get_MJPG_video_stream_from_serial_number(right_cam_sn)
    left_cam_stream = get_MJPG_video_stream_from_serial_number(left_cam_sn)
    center_cam_stream = get_MJPG_video_stream_from_serial_number(center_cam_sn)
except:
    pass

RioSerialConn = "/dev/ttyS0"

def generate_launch_description():
    current_pkg = FindPackageShare('king_engine')

    # Localization:
    cloud_node = Node(
            package='localization',
            executable='cloud',
            name='cloud',
        )
    
    imu_node = Node(
            package='localization',
            executable='imu',
            name='imu',
        )

    transformer_node = Node(
            package='localization',
            executable='transformer',
            name='transformer',
        )

    # # sick_scan_xd
    # sick_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('sick_scan_xd'),
    #             'launch',
    #             'sick_multiscan.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'hostname': '10.11.11.3',
    #         'udp_receiver_ip': '10.11.11.13',
    #     }.items()
    # )

    # sick_perception
    perception_launch = Node(
		name = 'obstacle_detection',
		package = 'sick_perception',
		executable = 'ldrp_node',
		output = 'screen',
		parameters = [PathJoinSubstitution([FindPackageShare('sick_perception'), 'config', 'params.yaml'])],
		remappings = [
			('scan', "/filtered_points"),
			('pose', "/adjusted_pose")
		]
	)

    # path_planning
    path_plan_launch = Node(
        package = 'path_plan',
        executable = 'nav_node',
        output = 'screen'
    )

    # king_engine
    king_engine_node = Node(
        package = 'king_engine',
        executable = 'main',
        output = 'screen',
    )

    # traversal
    traversal_node = Node(
        package = 'traversal',
        executable = 'main',
        output = 'screen',
    )

    # rio_interface
    rio_interface_node = Node(
        package = 'rio_interface',
        executable = "rio_interface",
        output = 'screen',
        parameters=[{ "serial_fd": RioSerialConn}]
    )

    # tf2
    # ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "scan"
    tf2_map_node = Node(
        package = 'tf2_ros',
        executable = "static_transform_publisher",
        output = 'screen',
        parameters = ["0.1", "0", "0", "0", "0.0", "0.0", "map", "world"],
    )

    tf2_odom_node = Node(
        package = 'tf2_ros',
        executable = "static_transform_publisher",
        output = 'screen',
        parameters = ["0.1", "0", "0", "0", "0.0", "0.0", "odom", "world"],
    )


    # video_publisher_node = Node(
    #     package = 'video_publisher',
    #     executable = 'vpub',
    #     output = 'screen',
    #     parameters=[{ "right_cam_path": right_cam_streav
    # m},
    #                 {"left_cam_path": left_cam_stream},
    #                 {"center_cam_path": center_cam_stream}]
    # )

    # 
    return LaunchDescription([
        cloud_node,
        imu_node,
        transformer_node,
        # sick_launch,
        perception_launch,
        path_plan_launch,
        king_engine_node,
        tf2_map_node,
        tf2_odom_node,
        traversal_node,
        # video_publisher_node
    ])
