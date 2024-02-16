import rclpy
from rclpy.node import Node

from king_engine.msg import Location                           
import time

class MinimalPublisher(Node):
    x: int
    y: int

    def __init__(self):
        super().__init__('localization')
        self.publisher_ = self.create_publisher(Location, 'location', 10) 
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0
        self.y = 0

    def timer_callback(self):
        msg = Location()                                                
        msg.x = float(self.x)
        msg.y = float(self.y)                                          
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ({msg.x}, {msg.y})')     
        self.x = self.x + 1
        self.y = self.y + 1  



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    #rclpy.spin(minimal_publisher)

    for i in range(0,122):
        msg = Location()                                                
        msg.x = float(i)
        msg.y = float(932)
        minimal_publisher.publisher_.publish(msg)
        time.sleep(2)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()