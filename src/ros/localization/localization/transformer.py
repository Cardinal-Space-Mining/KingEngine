import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation as Rot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import re
from localization.constants import aruco_positions, calibations
from threading import Thread
import time
import subprocess

class Transformer(Node):
    def __init__(self):
        super().__init__('overthruster') # type: ignore

        time.sleep(10) # Waiting for DLIO to launch
        
        self._logger.info(f"Starting aruco pose estimation")

        # SINED: get initial pose using aruco
        est = ArucoEstimator()
        try:
            while not est.sined():
                time.sleep(.1)
        except KeyboardInterrupt:
            est.kill()
            exit()
        position, orientation = est.get_init()
        self._logger.info(f"Finished aruco pose estimation")


        self.init_position = position
        self.init_orientation = orientation
        self.subscription = self.create_subscription(
            PoseStamped,
            '/dlio/odom_node/pose',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(PoseStamped, '/adjusted_pose', 10)

        self._logger.info(f"Started transformer node at position {position} and orientation {orientation}")

    def listener_callback(self, msg):
        position = (np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]) @ self.init_orientation) + self.init_position # type: ignore
        orientation = Rot.from_matrix(
            Rot.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]).as_matrix() @ self.init_orientation
        ).as_quat() # type: ignore

        new_msg = PoseStamped()

        new_msg.header = msg.header
        new_msg.header.stamp = self.get_clock().now().to_msg()

        new_msg.pose.orientation.x = orientation[0]
        new_msg.pose.orientation.y = orientation[1]
        new_msg.pose.orientation.z = orientation[2]
        new_msg.pose.orientation.w = orientation[3]
        
        new_msg.pose.position.x = position[0]
        new_msg.pose.position.y = position[1]
        new_msg.pose.position.z = position[2]

        self.publisher_.publish(new_msg)

class ArucoEstimator():
    def get_camera_info(self):
        endpoints = subprocess.run(("ls", "/dev"), capture_output=True).stdout.decode().split('\n')
        cameras = [f"/dev/{x}" for x in endpoints if re.compile('video\d+').match(x)]
        cams = []
        for camera in cameras:
            lines = subprocess.run(("v4l2-ctl", "-d", camera, "--info"), capture_output=True).stdout.decode().splitlines()
            for line in lines:
                if 'Serial' in line:
                    _, serial = line.replace('\t', '').replace(' ', '').split(':')
                    cams.append((serial, camera))
                    break
        return tuple(cams)    
    
    def __init__(self):
        self.init_orientation = None
        self.init_position = None

        for serial, camera in self.get_camera_info():
            Thread(target=self.camera_thead, args=(serial, camera)).start()
    
    def camera_thead(self, serial: str, camera: str):
        try:
            cap = cv2.VideoCapture(camera)
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 60)

            detector = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                cv2.aruco.DetectorParameters()
            )

            mtx = calibations[serial]['mtx']
            dist = calibations[serial]['dist']

            while self.init_position is None:
                _, frame = cap.read()
                
                corners, ids, rejected = detector.detectMarkers(frame)
                if corners == ():
                    continue
                flat_corners = ids.flatten().tolist()
                if flat_corners != [0, 1] and flat_corners != [1, 0] and flat_corners != [0]:
                    continue

                img_points = []
                real_points = []

                for i in range(0, len(ids)):
                    for i in range(0, 4):
                        img_points.append(corners[0][0][i])
                        real_points.append(aruco_positions[0][0][i])

                real_points = np.array(real_points).astype(np.float32)
                img_points = np.array(img_points).astype(np.float32)

                _, rvec, tvec = cv2.solvePnP(real_points, img_points, mtx, dist)

                Rt = cv2.Rodrigues(rvec)[0]
                R = Rt.transpose()
                pos = -R * tvec #type: ignore

                ZYX, jac = cv2.Rodrigues(rvec)
                totalrotmax = np.array([[ZYX[0, 0], ZYX[0, 1], ZYX[0, 2], tvec[0][0]], [ZYX[1, 0], ZYX[1, 1], ZYX[1, 2], tvec[1][0]], [ZYX[2, 0], ZYX[2, 1], ZYX[2, 2], tvec[2][0]], [0, 0, 0, 1]])
                inverserotmax = np.linalg.inv(totalrotmax)

                pitch = float(math.atan2(-R[2][1], R[2][2]))
                yaw = math.asin(R[2][0])
                roll = math.atan2(-R[1][0], R[0][0])
                x = inverserotmax[0][3]
                y = inverserotmax[1][3]
                z = inverserotmax[2][3]
                cap.release()

                self.init_position = np.array([x, y, z]) @ np.inv(calibations[serial]['rot']) + calibations[serial]['pos']
                self.init_orientation = Rot.from_euler('xyz', (roll, pitch, yaw), degrees=True).as_matrix() @ np.inv(calibations[serial]['rot']) # type: ignore
        except Exception as e:
            print(camera + " thread exception: " + str(e), end="")

    def sined(self) -> bool:
        return self.init_orientation is not None
    
    def get_init(self):
        return self.init_position, self.init_orientation

    def kill(self):
        self.init_position = 'die'

def main():
    rclpy.init()
    minimal_subscriber = Transformer()
    rclpy.spin(minimal_subscriber)
    print('DELIVERED')
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


# # SINED: get initial pose using aruco
# est = ArucoEstimator()
# try:
#     while not est.sined():
#         time.sleep(.1)
# except KeyboardInterrupt:
#     est.kill()
#     exit()
# position, orientation = est.get_init()
# print('SINED')

# # SEELED: start DLIO
# try:
#     cloud_filter = subprocess.Popen(
#         'python /home/gavin/CSM/localization_ws/cloud.py',
#         shell=True,
#         executable="/bin/bash"
#     )
#     imu_filter = subprocess.Popen(
#         'python /home/gavin/CSM/localization_ws/imu.py',
#         shell=True,
#         executable="/bin/bash"
#     )
#     dlio = subprocess.Popen(
#         'source /home/gavin/CSM/csmdlio_ws/install/setup.bash && ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false pointcloud_topic:=/filtered_cloud imu_topic:=/filtered_imu',
#         shell=True,
#         executable="/bin/bash"
#     )
#     print('SEELED')

# # DELIVERED: start pose transformation node
#     rclpy.init()
#     minimal_subscriber = Transformer()
#     rclpy.spin(minimal_subscriber)
#     print('DELIVERED')
#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()
# except:
#     subprocess.run(['kill', '-9 ', str(dlio.pid)])
#     subprocess.run(['kill', '-9 ', str(cloud_filter.pid)])
#     subprocess.run(['kill', '-9 ', str(imu_filter.pid)])