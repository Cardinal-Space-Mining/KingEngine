from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess
import usb_diff

#left, front, right
CAM_SERIALS = ['', '', '']
CAM_VIDS = ['', '', '']

def generate_launch_description():
    current_pkg = FindPackageShare('king_engine')

    # rio_interface
    rio_interface_node = Node(
        package = 'rio_interface',
        executable = 'rio_interface',
        output = 'screen',
        launch_arguments={
            'serial_fd' : '{SERIAL}'
        }.items()
    )

    for i in range(len(CAM_SERIALS)):
        CAM_VIDS[i] = usb_diff.get_MJPG_video_stream_from_serial_number(CAM_SERIALS[i])

    aruco_node = Node(
        package = 'aruco',
        executable = 'aruco',
        output = 'screen',
        launch_arguments={
            'cam_vid1' : CAM_VIDS[0],
            'cam_vid2' : CAM_VIDS[1],
            'cam_vid3' : CAM_VIDS[2]
        }
    )
    
    return LaunchDescription([
        rio_interface_node,
        aruco_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=aruco_node,
                on_exit=[
                    lambda:subprocess.run(tuple('ros2', 'launch', 'king_engine', 'lance.launch.py'))
                ]
            )
        )
    ])
