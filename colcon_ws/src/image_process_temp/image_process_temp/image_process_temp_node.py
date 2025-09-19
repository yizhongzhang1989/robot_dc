#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageProcessTempNode(Node):
    def __init__(self):
        super().__init__('image_process_temp_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_resized_topic', '/camera/image_resized')
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 0)  # 0 means keep aspect ratio
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_resized_topic = self.get_parameter('output_resized_topic').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        
        # Create QoS profile with RELIABLE policy
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber and publisher with RELIABLE QoS
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            reliable_qos
        )
        
        # Create publisher for resized images
        self.resized_image_publisher = self.create_publisher(
            Image,
            self.output_resized_topic,
            reliable_qos
        )
        
        self.get_logger().info('Image process temp node started')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Publishing resized images to: {self.output_resized_topic}')
        if self.resize_height == 0:
            self.get_logger().info(f'Resize dimensions: {self.resize_width}x? (keep aspect ratio)')
        else:
            self.get_logger().info(f'Resize dimensions: {self.resize_width}x{self.resize_height}')
    
    def image_callback(self, msg):
        """Callback function for processing incoming images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Calculate target size
            if self.resize_height == 0:
                # Keep aspect ratio - calculate height based on width
                original_height, original_width = cv_image.shape[:2]
                aspect_ratio = original_height / original_width
                target_height = int(self.resize_width * aspect_ratio)
                target_size = (self.resize_width, target_height)
            else:
                # Use specified dimensions
                target_size = (self.resize_width, self.resize_height)
            
            # Resize image
            resized_image = cv2.resize(cv_image, target_size)
            
            # Convert back to ROS Image message
            resized_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            resized_msg.header = msg.header
            
            # Publish resized image
            self.resized_image_publisher.publish(resized_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    image_process_temp_node = ImageProcessTempNode()
    
    try:
        rclpy.spin(image_process_temp_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_process_temp_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
