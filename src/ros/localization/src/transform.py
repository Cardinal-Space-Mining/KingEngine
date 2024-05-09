import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

initial_position = np.array([0, 0, 0])
initial_orientation = np.array([0, 0, 0])

class Transformer(Node):
    def __init__(self):
        super().__init__('overthruster')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/dlio/odom_node/pose',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(PoseStamped, '/adjusted_pose', 10)

    def listener_callback(self, msg):
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        orientation = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]).as_matrix()

        print()
        print(orientation)
        print(position)
    

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = Transformer()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()