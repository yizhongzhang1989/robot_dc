#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from common import get_temp_directory


class ImageProcessNode(Node):
    def __init__(self):
        super().__init__('image_process_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_resized_topic', '')  # Empty string means disabled
        self.declare_parameter('output_topic', '')  # Empty string means disabled
        self.declare_parameter('output_compressed_topic', '')  # Empty string means disabled
        self.declare_parameter('output_resized_compressed_topic', '')  # Empty string means disabled
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 0)  # 0 means keep aspect ratio
        self.declare_parameter('calibration_file', os.path.join(get_temp_directory(), 'camera_parameters', 'calibration_result.json'))
        self.declare_parameter('jpeg_quality', 85)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_resized_topic = self.get_parameter('output_resized_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_compressed_topic = self.get_parameter('output_compressed_topic').value
        self.output_resized_compressed_topic = self.get_parameter('output_resized_compressed_topic').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Enable flags based on whether topic names are provided
        self.enable_resized_output = bool(self.output_resized_topic.strip())
        self.enable_uncompressed_output = bool(self.output_topic.strip())
        self.enable_compressed_output = bool(self.output_compressed_topic.strip())
        self.enable_resized_compressed_output = bool(self.output_resized_compressed_topic.strip())
        self.enable_camera_info_output = self.enable_uncompressed_output  # Camera info pairs with uncompressed
        
        # Load camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration_parameters()
        
        # Create QoS profile optimized for real-time video streaming
        # Use RELIABLE delivery to make sure downstream consumers receive frames when possible
        # This helps ensure the image processing pipeline keeps data consistent across nodes
        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber and publisher
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            video_qos
        )
        
        # Create publishers only for enabled outputs
        self.resized_image_publisher = None
        self.image_publisher = None
        self.compressed_image_publisher = None
        self.resized_compressed_image_publisher = None
        self.camera_info_publisher = None
        
        if self.enable_resized_output:
            # Publisher for resized uncompressed images (original, not undistorted)
            self.resized_image_publisher = self.create_publisher(
                Image,
                self.output_resized_topic,
                video_qos
            )
        
        if self.enable_uncompressed_output:
            # Publisher for uncompressed images (for web viewer)
            self.image_publisher = self.create_publisher(
                Image,
                self.output_topic,
                video_qos
            )
        
        if self.enable_compressed_output:
            # Publisher for compressed images
            self.compressed_image_publisher = self.create_publisher(
                CompressedImage,
                self.output_compressed_topic,
                video_qos
            )
        
        if self.enable_resized_compressed_output:
            # Publisher for resized compressed images
            self.resized_compressed_image_publisher = self.create_publisher(
                CompressedImage,
                self.output_resized_compressed_topic,
                video_qos
            )
        
        if self.enable_camera_info_output:
            # Publisher for original camera info (intrinsic parameters)
            # Camera info can use default QoS as it's not high-frequency
            self.camera_info_publisher = self.create_publisher(
                CameraInfo,
                self.input_topic.replace('/image_', '/camera_info_'),
                10
            )
        
        self.get_logger().info('Image process node started')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        
        # Log only enabled outputs
        if self.enable_resized_output:
            self.get_logger().info(f'Publishing resized uncompressed to: {self.output_resized_topic}')
            if self.resize_height == 0:
                self.get_logger().info(f'Resize dimensions: {self.resize_width}x? (keep aspect ratio)')
            else:
                self.get_logger().info(f'Resize dimensions: {self.resize_width}x{self.resize_height}')
        if self.enable_uncompressed_output:
            self.get_logger().info(f'Publishing uncompressed to: {self.output_topic}')
        if self.enable_compressed_output:
            self.get_logger().info(f'Publishing compressed to: {self.output_compressed_topic}')
        if self.enable_resized_compressed_output:
            self.get_logger().info(f'Publishing resized compressed to: {self.output_resized_compressed_topic}')
            if not self.enable_resized_output:  # Only log resize dimensions if not already logged
                if self.resize_height == 0:
                    self.get_logger().info(f'Resize dimensions: {self.resize_width}x? (keep aspect ratio)')
                else:
                    self.get_logger().info(f'Resize dimensions: {self.resize_width}x{self.resize_height}')
        if self.enable_camera_info_output:
            self.get_logger().info(f'Publishing original camera info to: {self.input_topic.replace("/image_", "/camera_info_")}')
        
        # Log configuration
        if not any([self.enable_resized_output, self.enable_uncompressed_output, self.enable_compressed_output, self.enable_resized_compressed_output]):
            self.get_logger().warn('No output topics enabled! All topic parameters are empty.')
        
        self.get_logger().info(f'Using calibration file: {self.calibration_file}')
        if self.enable_compressed_output or self.enable_resized_compressed_output:
            self.get_logger().info(f'JPEG quality: {self.jpeg_quality}')
        
        # Variables for undistortion maps (computed once for efficiency)
        self.map1 = None
        self.map2 = None
        self.image_size = None
        
    def load_calibration_parameters(self):
        """Load camera calibration parameters from JSON file"""
        try:
            if not os.path.exists(self.calibration_file):
                self.get_logger().error(f'Calibration file not found: {self.calibration_file}')
                return
                
            with open(self.calibration_file, 'r') as f:
                calib_data = json.load(f)
            
            if not calib_data.get('success', False):
                self.get_logger().error('Calibration was not successful according to the file')
                return
                
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=np.float32)
            self.dist_coeffs = np.array(calib_data['distortion_coefficients'], dtype=np.float32)
            
            self.get_logger().info('Camera calibration parameters loaded successfully')
            self.get_logger().info(f'RMS Error: {calib_data.get("rms_error", "Unknown")}')
            self.get_logger().info(f'Image count used for calibration: {calib_data.get("image_count", "Unknown")}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration parameters: {str(e)}')
            
    def compute_undistortion_maps(self, image_size):
        """Compute undistortion maps for the given image size"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None, None

        # Compute optimal new camera matrix for undistortion only
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.dist_coeffs, 
            image_size, 
            0,  # alpha=0 means crop to remove black borders, alpha=1 means keeping all pixels
            image_size
        )
        
        # Compute undistortion maps
        map1, map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            new_camera_matrix,
            image_size,
            cv2.CV_16SC2
        )
        
        return map1, map2
        
    def image_callback(self, msg):
        """Callback function for processing incoming images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process resized original image FIRST (highest priority, simplest operation)
            if self.enable_resized_output and self.resized_image_publisher:
                self.publish_resized_image(cv_image, msg.header)  # Use original image, not undistorted
            
            # Only do undistortion processing if we need undistorted outputs
            if (self.enable_uncompressed_output or self.enable_compressed_output or 
                self.enable_resized_compressed_output):
                
                # Check if calibration parameters are available
                if self.camera_matrix is None or self.dist_coeffs is None:
                    self.get_logger().warn('No calibration parameters available, passing through original image')
                    undistorted_image = cv_image
                else:
                    # Get image dimensions
                    height, width = cv_image.shape[:2]
                    current_size = (width, height)
                    
                    # Compute undistortion maps if not done yet or image size changed
                    if (self.map1 is None or self.map2 is None or 
                        self.image_size != current_size):
                        self.image_size = current_size
                        self.map1, self.map2 = self.compute_undistortion_maps(current_size)
                        
                        if self.map1 is None or self.map2 is None:
                            self.get_logger().error('Failed to compute undistortion maps')
                            return
                            
                        self.get_logger().info(f'Computed undistortion maps for image size: {current_size}')
                    
                    # Apply undistortion using pre-computed maps
                    undistorted_image = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)
                
                # Convert back to ROS Image message (for web viewer)
                undistorted_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')
                
                # Copy header information
                undistorted_msg.header = msg.header
                
                # Publish uncompressed undistorted image (for web viewer) - only if enabled
                if self.enable_uncompressed_output and self.image_publisher:
                    self.image_publisher.publish(undistorted_msg)
                
                # Create and publish compressed image - only if enabled
                if self.enable_compressed_output and self.compressed_image_publisher:
                    self.publish_compressed_image(undistorted_image, msg.header)
                
                # Create and publish resized compressed image - only if enabled
                if self.enable_resized_compressed_output and self.resized_compressed_image_publisher:
                    self.publish_resized_compressed_image(undistorted_image, msg.header)
                
                # Publish original camera info - only if enabled
                if self.enable_camera_info_output and self.camera_info_publisher:
                    self.publish_original_camera_info(msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            
    def publish_compressed_image(self, cv_image, header):
        """Publish compressed JPEG image"""
        try:
            # Encode image as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            
            if not result:
                self.get_logger().error('Failed to encode image as JPEG')
                return
                
            # Create compressed image message
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()
            
            # Publish compressed image
            self.compressed_image_publisher.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error creating compressed image: {str(e)}')
            
    def publish_resized_image(self, cv_image, header):
        """Publish resized uncompressed image"""
        try:
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
            
            # Convert to ROS Image message
            resized_msg = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            resized_msg.header = header
            
            # Publish resized uncompressed image
            self.resized_image_publisher.publish(resized_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error creating resized image: {str(e)}')
            
    def publish_resized_compressed_image(self, cv_image, header):
        """Publish resized and compressed JPEG image"""
        try:
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
            
            # Encode image as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', resized_image, encode_param)
            
            if not result:
                self.get_logger().error('Failed to encode resized image as JPEG')
                return
                
            # Create compressed image message
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()
            
            # Publish resized compressed image
            self.resized_compressed_image_publisher.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error creating resized compressed image: {str(e)}')
            
    def publish_original_camera_info(self, header):
        """Publish original camera info (intrinsic parameters from calibration file)"""
        try:
            if self.camera_matrix is None or self.dist_coeffs is None:
                return
                
            # Create camera info message for original camera parameters
            camera_info = CameraInfo()
            camera_info.header = header
            camera_info.width = self.image_size[0] if self.image_size else 0
            camera_info.height = self.image_size[1] if self.image_size else 0
            
            # Original camera matrix (K)
            camera_info.k = [
                float(self.camera_matrix[0, 0]), 0.0, float(self.camera_matrix[0, 2]),
                0.0, float(self.camera_matrix[1, 1]), float(self.camera_matrix[1, 2]),
                0.0, 0.0, 1.0
            ]
            
            # Projection matrix (P) - same as K for single camera
            camera_info.p = [
                float(self.camera_matrix[0, 0]), 0.0, float(self.camera_matrix[0, 2]), 0.0,
                0.0, float(self.camera_matrix[1, 1]), float(self.camera_matrix[1, 2]), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Original distortion coefficients
            camera_info.d = [float(d) for d in self.dist_coeffs]
            camera_info.distortion_model = "plumb_bob"
            
            # Rectification matrix (identity for single camera)
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # Publish original camera info
            self.camera_info_publisher.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing original camera info: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    image_process_node = ImageProcessNode()
    
    try:
        rclpy.spin(image_process_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_process_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
