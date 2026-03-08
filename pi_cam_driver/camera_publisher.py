import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        # Name the node (this can be overridden by a launch file later)
        super().__init__('camera_publisher_node')

        # 1. Declare Parameters
        # This allows us to use the same script for the Left and Right Pi
        # just by passing different names when we launch it.
        self.declare_parameter('topic_name', 'camera/image_compressed')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('framerate', 30.0)

        topic_name = self.get_parameter('topic_name').value
        camera_id = self.get_parameter('camera_id').value
        fps = self.get_parameter('framerate').value

        # 2. Setup Publisher and OpenCV Bridge
        self.publisher_ = self.create_publisher(
            CompressedImage, topic_name, 10)
        self.br = CvBridge()

        # 3. Initialize Camera
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera {camera_id}")
            return

        # 4. Set up the publishing loop
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Publishing to {topic_name} at {fps} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Compress the OpenCV matrix into a JPEG ROS 2 message
            msg = self.br.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')

            # THE MOST IMPORTANT LINE FOR STEREO VISION:
            # We must stamp the header with the exact network-synced time
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Failed to grab frame from camera")

    def destroy_node(self):
        # Ensure we release the camera hardware when shutting down
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
