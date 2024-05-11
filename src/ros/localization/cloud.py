import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import time
import ros2_numpy as rnp
from scipy.spatial.transform import Rotation as R
import numpy as np

class CloudSub(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.r = R.from_euler('xyz', (0, 200, 240), degrees=True).as_matrix()
        print(self.r)
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            '/cloud_all_fields_fullframe',
            self.cloud_callback,
            10
        )
        self.publisher_ = self.create_publisher(PointCloud2, '/filtered_cloud', 10)

    def cloud_callback(self, msg):
        points = rnp.numpify(msg)
        points['xyz'] = points['xyz'] @ self.r
        new_msg = rnp.msgify(PointCloud2, points)
        new_msg.header.frame_id = 'world'
        new_msg.point_step = 12
        print(time.monotonic(), end=": ")
        print(len(msg.data))
        self.publisher_.publish(new_msg)
        

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = CloudSub()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()