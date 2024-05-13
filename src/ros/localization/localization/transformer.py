import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation as Rot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import re
from threading import Thread
import time
import subprocess

aruco_positions = [
    np.array([[
        [10, 0, 0],
        [190, 0, 0],
        [190, 180, 0],
        [10, 180, 0]
    ]]),
    np.array([[
        [0, 0, 190],
        [0, 0, 10],
        [0, 180, 10],
        [0, 180, 190]
    ]]),
]

calibations = {
    'YLAF20221208V1': {
        'mtx': np.array([
            [502.77264231,   0.,         322.89582315],
            [  0.,         502.78152803, 241.45546177],
            [  0.,           0.,           1.        ]
        ]),
        'dist': np.array([[ 0.19903941, -0.74796342,  0.00199075,  0.00354105,  0.87606197]]),
        'pos': np.array([-276.617, 11.617, 23.941]),
        'rot': Rot.from_euler('xyz', (-9.90, -20, 0), degrees=True).as_matrix()
    },
    'YLAF20221208V2': {
        'mtx': np.array([
            [521.06512431,   0.,         320.42556535],
            [  0.,         521.28236255, 242.55116183],
            [  0.,           0.,           1.        ]
        ]),
        'dist': np.array([[ 0.23879134, -0.94403776, -0.0017436, 0.00305515, 1.14485391]]),
        'pos': np.array([276.617, 11.617, 23.941]),
        'rot': Rot.from_euler('xyz', (-9.90, 20, 0), degrees=True).as_matrix()
    },
    'CSM15424': {
        'mtx': np.array([
            [525.08331471,   0.,         321.73028149],
            [  0.,         524.34561131, 244.12181265],
            [  0.,           0.,           1.        ]
        ]),
        'dist': np.array([[ 0.2334526, -0.87719634, 0.00268214, 0.00553653, 0.97017557]]),
        'pos': np.array([0, 43.353, 52.462]),
        'rot': Rot.from_euler('xyz', (-15, 0, 0), degrees=True).as_matrix()
    },
    '200901010001': {
        'mtx': np.array([
            [525.08331471,   0.,         321.73028149],
            [  0.,         524.34561131, 244.12181265],
            [  0.,           0.,           1.        ]
        ]),
        'dist': np.array([[ 0.2334526, -0.87719634, 0.00268214, 0.00553653, 0.97017557]]),
        'pos': np.array([0, 0, 0]),
        'rot': Rot.from_euler('xyz', (0, 0, 0), degrees=True).as_matrix()
    }
}

class Transformer(Node):
    def __init__(self, init_position, init_orientation):
        super().__init__('overthruster') # type: ignore
        self.init_position = init_position
        self.init_orientation = init_orientation
        self.subscription = self.create_subscription(
            PoseStamped,
            '/dlio/odom_node/pose',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(PoseStamped, '/adjusted_pose', 10)

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
        endpoints = subprocess.run(("v4l2-ctl", "--list-devices"), capture_output=True).stdout.decode().splitlines()
        cameras = []
        for line in endpoints:
            if line != '' and line[0] == '\t':
                cameras.append(line.replace('\t', ''))

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
        rclpy.logging.get_logger("aruco").info("starting threads")
        rclpy.logging.get_logger("aruco").info(str(self.get_camera_info()))
        for serial, camera in self.get_camera_info():
            Thread(target=self.camera_thead, args=(serial, camera)).start()
    
    def camera_thead(self, serial: str, camera: str):
        try:
            rclpy.logging.get_logger("aruco").info("opening camera")
            cap = cv2.VideoCapture(camera)
            
            rclpy.logging.get_logger("aruco").info("setting params")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 60)

            rclpy.logging.get_logger("aruco").info("dictionary")
            detector = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                cv2.aruco.DetectorParameters()
            )

            rclpy.logging.get_logger("aruco").info("reading calibration")
            mtx = calibations[serial]['mtx']
            dist = calibations[serial]['dist']

            while self.init_position is None:
                _, frame = cap.read()
                
                corners, ids, rejected = detector.detectMarkers(frame)
                if corners == ():
                    continue
                flat_corners = ids.flatten().tolist()
                if flat_corners == []:
                    continue

                img_points = []
                real_points = []

                for i in range(0, len(ids)):
                    for i in range(0, 4):
                        img_points.append(corners[0][0][i])
                        real_points.append(aruco_positions[0][0][i])

                print(img_points)

                real_points = np.array(real_points).astype(np.float32)
                img_points = np.array(img_points).astype(np.float32)

                _, rvec, tvec = cv2.solvePnP(real_points, img_points, mtx, dist)

                Rt = cv2.Rodrigues(rvec)[0]
                R = Rt.transpose()

                ZYX, _ = cv2.Rodrigues(rvec)
                totalrotmax = np.array([[ZYX[0, 0], ZYX[0, 1], ZYX[0, 2], tvec[0][0]], [ZYX[1, 0], ZYX[1, 1], ZYX[1, 2], tvec[1][0]], [ZYX[2, 0], ZYX[2, 1], ZYX[2, 2], tvec[2][0]], [0, 0, 0, 1]])
                inverserotmax = np.linalg.inv(totalrotmax)

                pitch = float(math.atan2(-R[2][1], R[2][2]))
                yaw = math.asin(R[2][0])
                roll = math.atan2(-R[1][0], R[0][0])
                x = inverserotmax[0][3]
                y = inverserotmax[1][3]
                z = inverserotmax[2][3]
                cap.release()

                self.init_orientation = Rot.from_euler('xyz', (roll, pitch, yaw), degrees=True).as_matrix() @ calibations[serial]['rot']
                self.init_position = np.array([x, y, z]) @ calibations[serial]['rot'] + calibations[serial]['pos']
        except Exception as e:
            rclpy.logging.get_logger("aruco").error(str(e))
            cv2.destroyAllWindows()
            

    def sined(self) -> bool:
        return self.init_orientation is not None
    
    def get_init(self):
        return self.init_position, self.init_orientation

    def kill(self):
        self.init_position = 'die'


def main():
    rclpy.logging.get_logger("aruco").info("aruco time!")

    # SINED: get initial pose using transformed aruco vals
    est = ArucoEstimator()
    try:
        while not est.sined():
            time.sleep(.1)
    except KeyboardInterrupt:
        est.kill()
        exit()
    position, orientation = est.get_init()
    # position = (position @ center_angle) + center_offset
    # orientation = orientation @ center_angle
    print(Rot.from_matrix(orientation).as_euler('xyz', degrees=True))
    rclpy.logging.get_logger("aruco").info("SINED")

    # SEELED: start DLIO
    try:
        dlio = subprocess.Popen(
            'source /home/po/dlio_ws/install/setup.bash && ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false pointcloud_topic:=/filtered_cloud imu_topic:=/filtered_imu',
            shell=True,
            executable="/bin/bash"
        )
        rclpy.logging.get_logger("aruco").info("SEELED")

    # DELIVERED: start pose transformation node
        rclpy.init()
        minimal_subscriber = Transformer(position, orientation)
        rclpy.spin(minimal_subscriber)
        rclpy.logging.get_logger("aruco").info("DELIVERED")
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(e)
        subprocess.run(['kill', '-9 ', str(dlio.pid)])

if __name__ == '__main__':
    main()