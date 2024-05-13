import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

IMX179_MJPG_settings = [
    {
        "resolution":(3264,2448),
        "fps":15
    },
    {
        "resolution":(2592,1944),
        "fps":15
    },{
        "resolution":(1920,1080),
        "fps":30
    },{
        "resolution":(1600,1200),
        "fps":30
    },{
        "resolution":(1280,720),
        "fps":30
    },{
        "resolution":(960,540),
        "fps":30
    },{
        "resolution":(848,480),
        "fps":30
    },{
        "resolution":(640,480),
        "fps":30
    },{
        "resolution":(640,360),
        "fps":30
    },{
        "resolution":(424,240),
        "fps":30
    },{
        "resolution":(320,240),
        "fps":30
    },{
        "resolution":(320,180),
        "fps":30
    }
]

class VideoPublisher(Node):

    __RIGHT_CAM_PARAM_NAME__ = "right_cam_path"
    __LEFT_CAM_PARAM_NAME__ = "left_cam_path"
    __CENTER_CAM_PARAM_NAME__ = "center_cam_path"



    __slots__ = ("RightCamPub", "LeftCamPub", "CtrCamPub", "right_capture","left_capture","center_capture", "bridge", "cam_setting", "right_cam_timer", "left_cam_timer", "center_cam_timer")


    def apply_mjpg_setting(self, cap, setting):
        self.get_logger().info(f"Provided Setting: {self.cam_setting}")
        cap.set(cv2.CAP_PROP_FPS, setting["fps"])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, setting["resolution"][0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, setting["resolution"][1])
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        self.get_logger().info(f"Actual Setting:")
        self.get_logger().info(f"Frame Size: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)} x {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
        self.get_logger().info(f"fps: {cap.get(cv2.CAP_PROP_FPS)}")
        self.get_logger().info(f"encoding: {cap.get(cv2.CAP_PROP_FOURCC)}")
        
    def load_camera(self, camera_file) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(camera_file)
        if not cap.isOpened():
                self.get_logger().error(f"Error: Unable to open camera at {camera_file}.")
                cap.release()
                raise RuntimeError("Could not open camera")

        return cap

    @staticmethod
    def get_cam_qos():
        qos_profile = rclpy.qos.qos_profile_sensor_data
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        qos_profile.history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        qos_profile.depth = 10
        qos_profile.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        qos_profile.avoid_ros_namespace_conventions = False
        qos_profile.deadline = rclpy.duration.Duration(seconds=1)
        return qos_profile

    def __init__(self):
        super().__init__('video_publisher')

        self.declare_parameter(VideoPublisher.__RIGHT_CAM_PARAM_NAME__, rclpy.Parameter.Type.STRING)
        self.declare_parameter(VideoPublisher.__LEFT_CAM_PARAM_NAME__, rclpy.Parameter.Type.STRING)
        self.declare_parameter(VideoPublisher.__CENTER_CAM_PARAM_NAME__, rclpy.Parameter.Type.STRING)
        
        self.bridge = CvBridge()

        self.RightCamPub = self.create_publisher(Image, 'ImageRight', VideoPublisher.get_cam_qos())
        self.LeftCamPub = self.create_publisher(Image, 'ImageLeft', VideoPublisher.get_cam_qos())
        self.CtrCamPub = self.create_publisher(Image, 'ImageCenter', VideoPublisher.get_cam_qos())

        right_cam_fd = self.get_parameter(VideoPublisher.__RIGHT_CAM_PARAM_NAME__).get_parameter_value().string_value
        left_cam_fd = self.get_parameter(VideoPublisher.__LEFT_CAM_PARAM_NAME__).get_parameter_value().string_value
        center_cam_fd = self.get_parameter(VideoPublisher.__CENTER_CAM_PARAM_NAME__).get_parameter_value().string_value

        self.right_capture = self.load_camera(right_cam_fd)
        self.left_capture = self.load_camera(left_cam_fd)
        self.center_capture = self.load_camera(center_cam_fd)

        self.cam_setting = IMX179_MJPG_settings[-5] #640x480 30fps

        self.apply_mjpg_setting(self.right_capture, self.cam_setting)
        self.apply_mjpg_setting(self.left_capture, self.cam_setting)
        self.apply_mjpg_setting(self.center_capture, self.cam_setting)

        self.right_cam_timer = self.create_timer(1.0 / self.cam_setting["fps"], lambda: VideoPublisher.publish_frame(self, self.right_capture, self.bridge, self.RightCamPub))
        self.left_cam_timer = self.create_timer(1.0 / self.cam_setting["fps"], lambda: VideoPublisher.publish_frame(self, self.left_capture, self.bridge, self.LeftCamPub))
        self.center_cam_timer = self.create_timer(1.0 / self.cam_setting["fps"], lambda: VideoPublisher.publish_frame(self, self.center_capture, self.bridge, self.CtrCamPub))

        self._logger.info(f"Launced video_publisher and bound cameras. Right: {right_cam_fd}. Left: {left_cam_fd}, Center: {center_cam_fd}")


    def destroy_node(self):
        self.right_capture.release()
        self.left_capture.release()
        self.center_capture.release()
        super().destroy_node()

    @staticmethod
    def publish_frame(node: Node, capture, bridge, publisher):
        ret, frame = capture.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            publisher.publish(msg)
        else:
            node.get_logger().error("No frame :(")
            
            

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    del video_publisher
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
