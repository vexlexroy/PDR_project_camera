import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # parameter for camera source (default: 0 = webcam)
        self.declare_parameter('camera_source', '0')
        source_param = self.get_parameter('camera_source').get_parameter_value().string_value

        # try to parse integer (webcam) or string (video path)
        try:
            self.camera_source = int(source_param)
        except ValueError:
            self.camera_source = source_param
            
        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera/video source: {self.camera_source}")
            raise RuntimeError("Camera open failed")

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        self.get_logger().info(f"Publishing images from source: {self.camera_source}")

        # Use a non-blocking loop to publish frames immediately
        self.timer = self.create_timer(0.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return  # skip if no frame

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
