import socket
import struct
import cv2
import numpy as np
import time
from collections import deque
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
FPS_AVERAGE_INTERVAL_SEC = 2

frame_buffer = {}
frame_info = {}
frame_last_update = {}
FRAME_TIMEOUT_SEC = 1.0  # Таймаут хранения "недособранных" кадров

def clean_old_frames():
    """Удаляет старые или зависшие кадры из буфера"""
    now = time.time()
    old_ids = [fid for fid, ts in frame_last_update.items() if now - ts > FRAME_TIMEOUT_SEC]
    for fid in old_ids:
        frame_buffer.pop(fid, None)
        frame_info.pop(fid, None)
        frame_last_update.pop(fid, None)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(5.0)

    prev_time = None
    frame_times = deque(maxlen=6000)
    frametimes_plot = deque(maxlen=200)

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    ax.set_xlabel('Frame')
    ax.set_ylabel('Frame time, ms')
    ax.set_title('Frame time (ms) over last 200 frames')
    ax.set_ylim(0, 100)
    ax.set_xlim(0, 200)

    print(f"Listening UDP {UDP_PORT}...")

    while True:
        clean_old_frames()
        try:
            data, addr = sock.recvfrom(1500)
        except socket.timeout:
            print("Timeout, no packets received")
            continue

        if len(data) < 16:
            continue
        try:
            frame_id, packet_id, num_packets, datalen, frame_len, _ = struct.unpack('<IHHHIH', data[:16])
        except struct.error:
            continue
        payload = data[16:]

        if len(payload) != datalen:
            continue
        if num_packets == 0 or packet_id >= num_packets:
            continue

        # Инициализация буфера для кадра
        if frame_id not in frame_buffer:
            frame_buffer[frame_id] = [None] * num_packets
            frame_info[frame_id] = {'size': frame_len, 'packets': num_packets, 'recv': 0}
        frame_last_update[frame_id] = time.time()

        # Сохраняем пакет (дубли не учитываем)
        if frame_buffer[frame_id][packet_id] is None:
            frame_buffer[frame_id][packet_id] = payload
            frame_info[frame_id]['recv'] += 1

        # Проверяем, собран ли кадр полностью
        if frame_info[frame_id]['recv'] == frame_info[frame_id]['packets']:
            jpg_bytes = b''.join(filter(None, frame_buffer[frame_id]))
            if len(jpg_bytes) == frame_info[frame_id]['size']:
                frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    now = time.time()
                    frame_times.append(now)
                    if prev_time is not None:
                        frametime = (now - prev_time) * 1000.0
                        frametimes_plot.append(frametime)
                    prev_time = now
                    while frame_times and now - frame_times[0] > FPS_AVERAGE_INTERVAL_SEC:
                        frame_times.popleft()
                    elapsed = (frame_times[-1] - frame_times[0]) if len(frame_times) > 1 else 1
                    avg_fps = (len(frame_times) - 1) / elapsed if elapsed > 0 else 0.0
                    cv2.putText(frame, f"AVG FPS ({FPS_AVERAGE_INTERVAL_SEC}s): {avg_fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                    cv2.imshow("ESP32-CAM UDP Stream", frame)
                    if len(frametimes_plot) > 1:
                        line.set_xdata(np.arange(len(frametimes_plot)))
                        line.set_ydata(list(frametimes_plot))
                        ax.set_ylim(0, max(100, max(frametimes_plot) * 1.2))
                        ax.set_xlim(0, len(frametimes_plot))
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                    if cv2.waitKey(1) == 27:
                        break
            # Чистим кадр (успешно или неуспешно)
            frame_buffer.pop(frame_id, None)
            frame_info.pop(frame_id, None)
            frame_last_update.pop(frame_id, None)

    cv2.destroyAllWindows()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()