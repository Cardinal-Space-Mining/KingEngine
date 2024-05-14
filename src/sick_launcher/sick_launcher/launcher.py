import rclpy
from rclpy import logging
from rclpy.node import Node
import subprocess
from threading import Event
import time

def main():
    rclpy.init()
    rclpy.logging.get_logger("sick_launcher").info("launching sick_scan_xd")
    try:
        sick_scan = subprocess.Popen(
            'ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=10.11.11.3 udp_receiver_ip:=10.11.11.13',
            shell=True,
            executable="/bin/bash"
        )
        rclpy.logging.get_logger("sick_launcher").info("started sick_scan_xd")
        Event().wait()
    except KeyboardInterrupt:
        rclpy.logging.get_logger("sick_launcher").info("killing sick_scan_xd")
        subprocess.run(['kill', '-9 ', str(sick_scan.pid)])
        time.sleep(2)
    except Exception as e:
        rclpy.logging.get_logger("sick_launcher").error(str(e))
        subprocess.run(['kill', '-9 ', str(sick_scan.pid)])

if __name__ == '__main__':
    main()