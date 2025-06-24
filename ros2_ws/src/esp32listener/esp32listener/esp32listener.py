import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import numpy as np
import cv2

class ESP32CamPublisher(Node):
    def __init__(self):
        super().__init__('esp32_cam_publisher')

        # Declare the IP parameter with a default value
        self.declare_parameter('esp_ip', '192.168.0.133')
        self.esp_ip = self.get_parameter('esp_ip').get_parameter_value().string_value
        self.url = f"http://{self.esp_ip}/stream"

        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info(f'ESP32CamPublisher node has been started. Using ESP32-CAM IP: {self.esp_ip}')
        
        self.publish_frames()

    def stream_jpeg(self):
        try:
            resp = requests.get(self.url, stream=True, timeout=(3, None))
            resp.raise_for_status()
        except requests.RequestException as e:
            self.get_logger().error(f"Failed to connect to ESP32-CAM at {self.url}: {e}")
            return

        buf = b""
        for chunk in resp.iter_content(chunk_size=4096):
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2)
                if start != -1 and end != -1:
                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]
                    yield jpg
                else:
                    break

    def publish_frames(self):
        for jpg in self.stream_jpeg():
            frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
