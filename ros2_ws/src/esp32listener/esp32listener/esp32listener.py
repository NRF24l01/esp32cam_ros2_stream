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
frame_last_update = {}
FRAME_TIMEOUT_SEC = 1.0  # Максимальное время жизни "недособранного" кадра

class UdpCamImageNode(Node):
    def __init__(self):
        super().__init__('esp32listener')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.001)
        self.frame_times = deque(maxlen=6000)
        self.prev_time = None

        self.get_logger().info(f"Listening UDP on {UDP_PORT}...")

        # timer для периодического опроса сокета
        self.timer = self.create_timer(0.001, self.recv_and_publish)

    def clean_old_frames(self):
        now = time.time()
        old_ids = [fid for fid, ts in frame_last_update.items() if now - ts > FRAME_TIMEOUT_SEC]
        for fid in old_ids:
            frame_buffer.pop(fid, None)
            frame_info.pop(fid, None)
            frame_last_update.pop(fid, None)

    def recv_and_publish(self):
        # Чистим старые кадры перед приёмом новых
        self.clean_old_frames()
        try:
            data, addr = self.sock.recvfrom(1500)
        except (socket.timeout, BlockingIOError):
            return
        except Exception as e:
            self.get_logger().warn(f"Socket error: {e}")
            return

        if len(data) < 16:
            return

        try:
            frame_id, packet_id, num_packets, datalen, frame_len, _ = struct.unpack('<IHHHIH', data[:16])
        except struct.error:
            return
        payload = data[16:]

        if len(payload) != datalen:
            return
        if num_packets == 0 or packet_id >= num_packets:
            return

        # Сборка кадров (независимые буферы по frame_id)
        if frame_id not in frame_buffer:
            frame_buffer[frame_id] = [None] * num_packets
            frame_info[frame_id] = {'size': frame_len, 'packets': num_packets, 'recv': 0}
        frame_last_update[frame_id] = time.time()

        if frame_buffer[frame_id][packet_id] is None:
            frame_buffer[frame_id][packet_id] = payload
            frame_info[frame_id]['recv'] += 1

        if frame_info[frame_id]['recv'] == frame_info[frame_id]['packets']:
            jpg_bytes = b''.join(filter(None, frame_buffer[frame_id]))
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

            # Очистка всех буферов для этого кадра
            frame_buffer.pop(frame_id, None)
            frame_info.pop(frame_id, None)
            frame_last_update.pop(frame_id, None)


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