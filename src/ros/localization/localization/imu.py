import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.r = R.from_euler('xyz', (0, 200, 240), degrees=True).as_matrix()
        self.subscription = self.create_subscription(
            Imu,
            '/sick_scansegment_xd/imu',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Imu, '/filtered_imu', 10)
        self.header = None
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None
        self._logger.info(f"Localization.imu initialized")

    def listener_callback(self, msg):
        self.header = msg.header
        self.orientation = R.from_matrix(
            R.from_quat([
                msg.orientation.x, 
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]).as_matrix() @ self.r
        ).as_quat()
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]) @ self.r
        self.linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]) # @ self.r

        new_msg = Imu()
        
        # msg.header = self.header
        
        new_msg.header = msg.header
        new_msg.header.stamp = self.get_clock().now().to_msg()

        print(time.monotonic())

        new_msg.orientation.x = self.orientation[0]
        new_msg.orientation.y = self.orientation[1]
        new_msg.orientation.z = self.orientation[2]
        new_msg.orientation.w = self.orientation[3]
        
        new_msg.angular_velocity.x = self.angular_velocity[0]
        new_msg.angular_velocity.y = self.angular_velocity[1]
        new_msg.angular_velocity.z = self.angular_velocity[2]
        
        new_msg.linear_acceleration.x = self.linear_acceleration[0]
        new_msg.linear_acceleration.y = self.linear_acceleration[1]
        new_msg.linear_acceleration.z = self.linear_acceleration[2]

        self.publisher_.publish(new_msg)

    def timer_callback(self):
        pass
        msg = Imu()
        
        msg.header = self.header

        msg.orientation.x = self.orientation[0]
        msg.orientation.y = self.orientation[1]
        msg.orientation.z = self.orientation[2]
        msg.orientation.w = self.orientation[3]
        
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]
        
        msg.linear_acceleration.x = self.linear_acceleration[0]
        msg.linear_acceleration.y = self.linear_acceleration[1]
        msg.linear_acceleration.z = self.linear_acceleration[2]

        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = ImuSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()