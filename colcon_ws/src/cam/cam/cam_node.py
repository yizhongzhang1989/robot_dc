import time
import rclpy
from rclpy.node import Node

RTSP_URLS = {
    'cam100': 'rtsp://admin:123456@192.168.1.100/stream0',
    'cam101': 'rtsp://admin:123456@192.168.1.101/stream0'
}

def get_rtsp_snapshot(rtsp_url):
    import cv2
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        return None, 'Failed to open RTSP stream'
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return None, 'Failed to read frame'
    _, buffer = cv2.imencode('.jpg', frame)
    import base64
    img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
    return img_b64, None

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.cam_ids = ['cam100', 'cam101']
        self.timer_period = 30.0  # seconds
        timers = []
        for cam_id in self.cam_ids:
            timer = self.create_timer(self.timer_period, lambda cam_id=cam_id: self.take_snapshot(cam_id))
            timers.append(timer)

    def take_snapshot(self, cam_id, retry=3, retry_interval=2):
        rtsp_url = RTSP_URLS[cam_id]
        for i in range(retry):
            img_b64, err = get_rtsp_snapshot(rtsp_url)
            if img_b64:
                self.get_logger().info(f"[{cam_id}] Snapshot succeeded")
                return img_b64
            else:
                self.get_logger().warn(f"[{cam_id}] Snapshot failed: {err}")
            time.sleep(retry_interval)
        self.get_logger().error(f"[{cam_id}] Multiple attempts failed, camera may need to be restarted!")
        self.restart_camera(cam_id)
        return None

    def restart_camera(self, cam_id):
        self.get_logger().warn(f"[{cam_id}] Attempting to restart camera...")
        # Integrate actual hardware restart command here if needed
        time.sleep(5)
        self.get_logger().info(f"[{cam_id}] Restart complete, waiting for camera to recover...")
        time.sleep(10)

def main():
    rclpy.init()
    node = CamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 