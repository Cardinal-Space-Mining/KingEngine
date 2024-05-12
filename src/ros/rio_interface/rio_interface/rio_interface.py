from threading import Lock
import rclpy
from rclpy.node import Node
from custom_types.srv import SetTrackVelocity, StartMining, StopMining, StartOffload, TerminateRobot

from .PiSerialControl.serial_api import SerialManager

class RioInterface(Node):

    __SERIAL_FD_PARAM__ = "serial_fd"

    __slots__ = ("service1", "service2", "service3", "service4", "serial_lock", "serial_ctrler")

    def __init__(self):
        super().__init__('rio_interface')

        self.declare_parameter(RioInterface.__SERIAL_FD_PARAM__, rclpy.Parameter.Type.STRING)


        self.service1 = self.create_service(SetTrackVelocity, 'set_track_velocity', self.set_track_velo_callback)
        self.service2 = self.create_service(StartMining, 'start_mining', self.start_mining_callback)
        self.service3 = self.create_service(StopMining, 'stop_mining', self.stop_mining_callback)
        self.service4 = self.create_service(StartOffload, 'start_offload', self.start_offload_callback)
        self.service5 = self.create_service(TerminateRobot, 'terminate_robot', self.terminate_robot_callback)

        # I am using this lock because I am not entirely sure about ROS2's threading model when dealing with the lower layers.
        self.serial_lock = Lock()
        device = self.get_parameter(RioInterface.__SERIAL_FD_PARAM__).get_parameter_value().string_value
        self.serial_ctrler = SerialManager(device)

        self._logger.info(f"Launched rio_interface and bound to device {device}")

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
        self._logger.debug(f"Stopped Offload")
    
    def terminate_robot(self, _, response):
        with self.serial_lock:
            response.return_value = self.serial_ctrler._send_msg_func(SerialManager.STOP_MINING_FUNC_NUM) or self.serial_ctrler._send_msg_func(SerialManager.STOP_OFFLOAD_FUNC_NUM)
            for i in range(5):
                response.return_value = response.return_value or self.serial_ctrler._send_msg(SerialManager.MOTOR_CTRL, i, 0)
        self._logger.debug(f"Robot Terminated")
def main(args=None):
    rclpy.init(args=args)

    rio_interface = RioInterface()

    rclpy.spin(rio_interface)

    rio_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
