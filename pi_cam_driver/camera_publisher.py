import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')

        self.declare_parameter('topic_name', '/left/camera/image/compressed')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('framerate', 30.0)

        self.declare_parameter('jpeg_quality', 30)

        topic_name = self.get_parameter('topic_name').value
        camera_id = self.get_parameter('camera_id').value
        fps = self.get_parameter('framerate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.publisher_ = self.create_publisher(
            CompressedImage, topic_name, qos_profile_sensor_data)

        self.proc = None
        self._latest_jpeg = None
        self._jpeg_lock = threading.Lock()
        self._reader_thread = None

        if not self._try_libcamera(camera_id, fps):
            self.get_logger().error(f"Failed to open camera {camera_id}")
            return

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Publishing to {topic_name} at {fps} FPS (QoS: Best Effort, Quality: {self.jpeg_quality})")

    def _try_libcamera(self, camera_id, fps):
        cmd = None
        if shutil.which('rpicam-vid'):
            cmd = 'rpicam-vid'
        elif shutil.which('libcamera-vid'):
            cmd = 'libcamera-vid'

        if cmd is None:
            self.get_logger().warn("Neither rpicam-vid nor libcamera-vid found")
            return False

        try:
            self.proc = subprocess.Popen(
                [
                    cmd, '-t', '0',
                    '--camera', str(camera_id),
                    '--width', '640', '--height', '480',
                    '--framerate', str(int(fps)),
                    '--codec', 'mjpeg',
                    '-q', str(self.jpeg_quality),
                    '--nopreview',
                    '-o', '-'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            self._reader_thread = threading.Thread(
                target=self._read_mjpeg_stream, daemon=True
            )
            self._reader_thread.start()

            time.sleep(2.0)
            if self.proc.poll() is not None:
                stderr_out = self.proc.stderr.read().decode(errors='replace')
                self.get_logger().warn(f"{cmd} exited early: {stderr_out}")
                self.proc = None
                return False

            self.get_logger().info(f"Started {cmd} successfully")
            return True
        except FileNotFoundError:
            self.get_logger().warn(f"{cmd} not found")
            return False

    def _read_mjpeg_stream(self):
        buf = bytearray()
        stdout = self.proc.stdout
        while self.proc and self.proc.poll() is None:
            chunk = stdout.read(4096)
            if not chunk:
                break
            buf.extend(chunk)
            while True:
                start = buf.find(b'\xff\xd8')
                if start == -1:
                    buf.clear()
                    break
                end = buf.find(b'\xff\xd9', start + 2)
                if end == -1:
                    if start > 0:
                        del buf[:start]
                    break
                jpeg_data = bytes(buf[start:end + 2])
                with self._jpeg_lock:
                    self._latest_jpeg = jpeg_data
                del buf[:end + 2]

    def timer_callback(self):
        with self._jpeg_lock:
            jpeg_data = self._latest_jpeg
            self._latest_jpeg = None

        if jpeg_data is not None:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            msg.format = "jpeg"
            msg.data = jpeg_data
            self.publisher_.publish(msg)

    def destroy_node(self):
        if self.proc is not None:
            self.proc.terminate()
            self.proc.wait()
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
