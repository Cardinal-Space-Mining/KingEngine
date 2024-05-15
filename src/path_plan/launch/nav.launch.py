from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	current_pkg = FindPackageShare('path_plan')

	obstacles_topic_cfg = LaunchConfiguration('obstacles_topic', default='/ldrp/obstacle_grid')
	declare_pointcloud_topic_arg = DeclareLaunchArgument(
		'obstacles_topic',
		default_value = obstacles_topic_cfg,
		description = 'Input obstacle grid topic name'
	)

	pose_topic_cfg = LaunchConfiguration('pose_topic', default = 'adjusted_pose')
	# pose_topic_cfg = LaunchConfiguration('pose_topic', default = '/uesim/pose')
	declare_pose_topic_arg = DeclareLaunchArgument(
		'pose_topic',
		default_value = pose_topic_cfg,
		description = 'Input current pose topic name'
	)

	destination_topic_cfg = LaunchConfiguration('destination_topic', default = 'target_pose')
	declare_destination_topic_arg = DeclareLaunchArgument(
		'destination_topic',
		default_value = destination_topic_cfg,
		description = 'Input destination pose topic name'
	)

	params_yaml_path = PathJoinSubstitution([current_pkg, 'config', 'params.yaml'])

	nav_node = Node(
		name = 'nav',
		package = 'path_plan',
		executable = 'nav_node',
		output = 'screen',
		parameters = [params_yaml_path],
		remappings = [
			('obstacle_grid', obstacles_topic_cfg),
			('current_pose', pose_topic_cfg),
			('target_pose', destination_topic_cfg)
		]
	)

	return LaunchDescription([
		declare_pointcloud_topic_arg,
		declare_pose_topic_arg,
		declare_destination_topic_arg,
		nav_node
	])
