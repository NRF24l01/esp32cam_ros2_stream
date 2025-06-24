import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import numpy as np
import cv2

ESP_IP = "192.168.0.133" # ваш IP
URL = f"http://{ESP_IP}/stream"

class ESP32CamPublisher(Node):
    def __init__(self):
        super().__init__('esp32_cam_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.get_logger().info('ESP32CamPublisher node has been started.')
        self.publish_frames()

    def stream_jpeg(self):
        resp = requests.get(URL, stream=True, timeout=(3, None))
        resp.raise_for_status()
        buf = b""

        for chunk in resp.iter_content(chunk_size=4096):
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2)
                if start != -1 and end != -1:
                    jpg = buf[start:end+2]
                    buf = buf[end+2:]
                    yield jpg
                else:
                    break

    def publish_frames(self):
        """
        Continuously reads JPEG frames from the ESP32-CAM stream and publishes them
        to the 'image_raw' topic as sensor_msgs/Image messages.
        """
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
    import cv2
    main()
