import rclpy
from rclpy.node import Node

from threading import Thread
from rclpy import qos
from rclpy.executors import MultiThreadedExecutor
from localization.localization_main import localization_main
from custom_types.msg import Location                           
import time

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    loc_node = Node('localization')
    executor.add_node(loc_node)

    loc_publisher = loc_node.create_publisher(Location, 'location', 10) 

    thd = Thread(target=lambda: executor.spin)
    thd.start()

    def publish_location(x: float, y: float) -> None:
        msg = Location()                                                
        msg.x = float(x)
        msg.y = float(y)                                          
        loc_publisher.publish(msg)
        loc_node.get_logger().info(f'Publishing: ({msg.x}, {msg.y})')  

    # Hand the main thread off to localization
    try:
        localization_main(publish_location)
    finally:
        loc_publisher.destroy()
        loc_node.destroy_node()
        thd.join(0.1)
        rclpy.shutdown()
    



if __name__ == '__main__':
    main()