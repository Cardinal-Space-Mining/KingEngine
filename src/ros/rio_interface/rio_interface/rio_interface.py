from threading import Lock
import rclpy
from rclpy.node import Node
from custom_types.srv import SetTrackVelocity, StartMining, StopMining, StartOffload

from .PiSerialControl.serial_api import SerialManager

class RioInterface(Node):

    __slots__ = ("service1", "service2", "service3", "service4", "serial_lock", "serial_ctrler")

    def __init__(self):
        super().__init__('rio_interface')

        self.service1 = self.create_service(SetTrackVelocity, 'set_track_velocity', self.set_track_velo_callback)
        self.service2 = self.create_service(StartMining, 'start_mining', self.start_mining_callback)
        self.service3 = self.create_service(StopMining, 'stop_mining', self.stop_mining_callback)
        self.service4 = self.create_service(StartOffload, 'start_offload', self.start_offload_callback)

        # I am using this lock because I am not entirely sure about ROS2's threading model when dealing with the lower layers.
        self.serial_lock = Lock()

        self.serial_ctrler = SerialManager("TODO")

        self._logger.info("Launched rio_interface")

    def set_track_velo_callback(self, request, response):
        with self.serial_lock:
            response.return_value = self.serial_ctrler._send_msg(SerialManager.MOTOR_CTRL, request.motor_number, request.velocity_turns_per_second)
        self._logger.debug(f"Handled set_track_velo_callback. Motor Number: {request.motor_number}. Velocity: {request.velocity_turns_per_second}")

    def start_mining_callback(self, _, response):
        with self.serial_lock:
            response.return_value = self.serial_ctrler._send_msg_func(SerialManager.START_MINING_FUNC_NUM)
        self._logger.debug(f"Started Mining")

    def stop_mining_callback(self, _, response):
        with self.serial_lock:
            response.return_value = self.serial_ctrler._send_msg_func(SerialManager.STOP_MINING_FUNC_NUM)
        self._logger.debug(f"Stopped Mining")

    def start_offload_callback(self, _, response):
        with self.serial_lock:
            response.return_value = self.serial_ctrler._send_msg_func(SerialManager.STOP_OFFLOAD_FUNC_NUM)
        self._logger.debug(f"Stopped Mining")
def main(args=None):
    rclpy.init(args=args)

    rio_interface = RioInterface()

    rclpy.spin(rio_interface)

    rio_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
