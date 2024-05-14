from launch import LaunchDescription
import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

RioSerialConn = "/dev/ttyS0"

def generate_launch_description():
    current_pkg = FindPackageShare('king_engine')

    # sick_launcher
    sick_node = Node(
        package='sick_launcher',
        executable='launcher',
        name='launcher',
    )

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

    # perception
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

    # video_publisher_node = Node(
    #     package = 'video_publisher',
    #     executable = 'vpub',
    #     output = 'screen',
    #     parameters=[{ "right_cam_path": right_cam_streav
    # m},
    #                 {"left_cam_path": left_cam_stream},
    #                 {"center_cam_path": center_cam_stream}]
    # )

    return LaunchDescription([
        sick_node,
        cloud_node,
        imu_node,
        transformer_node,
        perception_launch,
        path_plan_launch,
        king_engine_node,
        tf2_map_node,
        tf2_odom_node,
        traversal_node,
        # video_publisher_node
    ])
