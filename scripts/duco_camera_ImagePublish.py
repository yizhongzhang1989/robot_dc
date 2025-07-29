import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import time

class RTSPCameraPublisher(Node):
    def __init__(self):
        super().__init__("rtsp_camera_publisher")

        # 👉 替换成你自己的 RTSP 地址
        self.stream_url = "rtsp://admin:123456@192.168.1.102/stream0"
        
        # 创建视频捕获对象并优化低延迟参数
        self.cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        
        # 设置低延迟参数
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # 完全禁用缓冲区
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率
        
        # 尝试设置其他低延迟参数
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
        except:
            pass

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open RTSP stream. Please check the URL.")
            return

        self.image_pub = self.create_publisher(Image, "/rtsp_camera/image_raw", 1)
        self.bridge = CvBridge()
        
        # 事件驱动相关变量
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self._last_warn_time = 0
        self._consecutive_failures = 0
        
        # 事件驱动核心：帧更新事件
        self.frame_ready_event = threading.Event()
        self.frame_id = 0
        self.last_published_frame_id = -1
        
        # 性能监控变量
        self.frame_count = 0
        self.publish_count = 0
        self.start_time = time.time()
        
        # 启动帧读取线程
        self.read_thread = threading.Thread(target=self._read_frames, daemon=True)
        self.read_thread.start()
        
        # 启动事件驱动发布线程
        self.publish_thread = threading.Thread(target=self._event_driven_publish, daemon=True)
        self.publish_thread.start()
        
        # 等待第一帧
        self.get_logger().info("Waiting for the first frame...")
        start_time = time.time()
        while self.latest_frame is None and time.time() - start_time < 5.0:
            time.sleep(0.1)
        
        if self.latest_frame is None:
            self.get_logger().warn("Can not get the first frame. Please check the RTSP stream.")
        else:
            self.get_logger().info("Get the first frame successfully.")
        
        # 启动性能监控定时器
        self.monitor_timer = self.create_timer(5.0, self.print_performance_stats)
        
        self.get_logger().info("Event-driven publisher is ready to publish images from RTSP stream.")

    def _read_frames(self):
        """持续读取帧，新帧到达时立即触发发布事件"""
        while self.running:
            frame_start_time = time.time()
            ret, frame = self.cap.read()
            
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.frame_id += 1
                    self.frame_count += 1
                
                # 新帧到达，立即触发发布事件
                self.frame_ready_event.set()
                self._consecutive_failures = 0
                
            else:
                self._consecutive_failures += 1
                if self._consecutive_failures > 3:
                    self.get_logger().warn("Trying to reconnect to RTSP stream...")
                    self._reconnect()
                    self._consecutive_failures = 0
                time.sleep(0.01)

    def _event_driven_publish(self):
        """事件驱动的发布循环 - 只在新帧到达时发布"""
        while self.running:
            # 等待新帧事件，最长等待100ms避免死锁
            if self.frame_ready_event.wait(timeout=0.1):
                self.frame_ready_event.clear()
                
                # 检查是否有新帧需要发布
                if self.frame_id > self.last_published_frame_id:
                    self._publish_latest_frame()
                    self.last_published_frame_id = self.frame_id

    def _publish_latest_frame(self):
        """发布最新帧"""
        frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
        
        if frame is None:
            current_time = time.time()
            if current_time - self._last_warn_time > 2.0:
                self.get_logger().warn("Cannot get a valid frame from the camera.")
                self._last_warn_time = current_time
            return

        try:
            # 添加时间戳到图像消息
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "camera_frame"
            
            self.image_pub.publish(image_msg)
            self.publish_count += 1
            
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def _reconnect(self):
        """重新连接RTSP流"""
        try:
            if self.cap.isOpened():
                self.cap.release()
            
            self.cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # 保持零缓冲
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if not self.cap.isOpened():
                self.get_logger().error("Reconnect failed. Cannot open RTSP stream.")
            else:
                self.get_logger().info("Reconnected to RTSP stream successfully.")
        except Exception as e:
            self.get_logger().error(f"Error during reconnection: {e}")

    def print_performance_stats(self):
        """打印性能统计信息"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time > 0:
            frame_fps = self.frame_count / elapsed_time
            publish_fps = self.publish_count / elapsed_time
            efficiency = (self.publish_count / max(self.frame_count, 1)) * 100
            
            self.get_logger().info(
                f"Performance: Frames={frame_fps:.1f}fps, "
                f"Published={publish_fps:.1f}fps, "
                f"Efficiency={efficiency:.1f}%"
            )
            
            # 重置计数器
            self.frame_count = 0
            self.publish_count = 0
            self.start_time = current_time

    def destroy_node(self):
        """清理资源"""
        self.running = False
        
        # 触发事件以便发布线程能够退出
        self.frame_ready_event.set()
        
        # 等待线程结束
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
        if hasattr(self, 'publish_thread'):
            self.publish_thread.join(timeout=1.0)
            
        if hasattr(self, 'cap'):
            self.cap.release()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RTSPCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()