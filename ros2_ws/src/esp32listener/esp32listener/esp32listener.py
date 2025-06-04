import socket
import struct
import cv2
import numpy as np
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
FPS_AVERAGE_INTERVAL_SEC = 2

frame_buffer = {}
frame_info = {}

class UdpCamImageNode(Node):
    def __init__(self):
        super().__init__('esp32listener')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(5.0)
        self.frame_times = deque(maxlen=6000)
        self.prev_time = None

        self.get_logger().info(f"Listening UDP on {UDP_PORT}...")

        # timer для периодического опроса сокета
        self.timer = self.create_timer(0.001, self.recv_and_publish)

    def recv_and_publish(self):
        try:
            data, addr = self.sock.recvfrom(1500)
        except socket.timeout:
            return
        except BlockingIOError:
            return

        if len(data) < 16:
            return

        frame_id, packet_id, num_packets, datalen, frame_len, _ = struct.unpack('<IHHHIH', data[:16])
        payload = data[16:]

        if len(payload) != datalen:
            return

        # Сборка кадров
        if frame_id not in frame_buffer:
            frame_buffer[frame_id] = [None] * num_packets
            frame_info[frame_id] = {'size': frame_len, 'packets': num_packets, 'recv': 0}

        if frame_buffer[frame_id][packet_id] is None:
            frame_buffer[frame_id][packet_id] = payload
            frame_info[frame_id]['recv'] += 1

        if frame_info[frame_id]['recv'] == frame_info[frame_id]['packets']:
            jpg_bytes = b''.join(frame_buffer[frame_id])
            if len(jpg_bytes) == frame_info[frame_id]['size']:
                frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    now = time.time()
                    self.frame_times.append(now)
                    self.prev_time = now

                    # Публикация в ROS2 топик
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_.publish(image_msg)

            # очистка
            del frame_buffer[frame_id]
            del frame_info[frame_id]


def main(args=None):
    rclpy.init(args=args)
    node = UdpCamImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()