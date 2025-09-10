#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os


class ImageProcessNode(Node):
    def __init__(self):
        super().__init__('image_process_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_undistorted')
        self.declare_parameter('output_compressed_topic', '/camera/image_undistorted/compressed')
        self.declare_parameter('output_resized_compressed_topic', '/camera/image_undistorted_resize_jpeg')
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)
        self.declare_parameter('calibration_file', '/home/a/Documents/robot_dc2/temp/calibration_result.json')
        self.declare_parameter('jpeg_quality', 85)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_compressed_topic = self.get_parameter('output_compressed_topic').value
        self.output_resized_compressed_topic = self.get_parameter('output_resized_compressed_topic').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Load camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration_parameters()
        
        # Create subscriber and publisher
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        # Publisher for uncompressed images (for web viewer)
        self.image_publisher = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        # Publisher for compressed images
        self.compressed_image_publisher = self.create_publisher(
            CompressedImage,
            self.output_compressed_topic,
            10
        )
        
        # Publisher for resized compressed images
        self.resized_compressed_image_publisher = self.create_publisher(
            CompressedImage,
            self.output_resized_compressed_topic,
            10
        )
        
        # Publishers for camera info (intrinsic parameters)
        self.undistorted_camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.output_topic.replace('/image_', '/camera_info_'),
            10
        )
        
        self.resized_camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.output_resized_compressed_topic.replace('/image_', '/camera_info_'),
            10
        )
        
        self.get_logger().info('Image process node started')
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Publishing uncompressed to: {self.output_topic}')
        self.get_logger().info(f'Publishing compressed to: {self.output_compressed_topic}')
        self.get_logger().info(f'Publishing resized compressed to: {self.output_resized_compressed_topic}')
        self.get_logger().info(f'Publishing undistorted camera info to: {self.output_topic.replace("/image_", "/camera_info_")}')
        self.get_logger().info(f'Publishing resized camera info to: {self.output_resized_compressed_topic.replace("/image_", "/camera_info_")}')
        self.get_logger().info(f'Resize dimensions: {self.resize_width}x{self.resize_height}')
        self.get_logger().info(f'Using calibration file: {self.calibration_file}')
        self.get_logger().info(f'JPEG quality: {self.jpeg_quality}')
        
        # Variables for undistortion maps (computed once for efficiency)
        self.map1 = None
        self.map2 = None
        self.image_size = None
        
        # Variables for camera info messages
        self.undistorted_camera_matrix = None
        self.resized_camera_matrix = None
        
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
            
        # Compute optimal new camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.dist_coeffs, 
            image_size, 
            1,  # alpha=1 means keep all pixels
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
        
        # Store the new camera matrix for undistorted images
        self.undistorted_camera_matrix = new_camera_matrix
        
        # Compute resized camera matrix
        scale_x = self.resize_width / image_size[0]
        scale_y = self.resize_height / image_size[1]
        self.resized_camera_matrix = new_camera_matrix.copy()
        self.resized_camera_matrix[0, 0] *= scale_x  # fx
        self.resized_camera_matrix[1, 1] *= scale_y  # fy
        self.resized_camera_matrix[0, 2] *= scale_x  # cx
        self.resized_camera_matrix[1, 2] *= scale_y  # cy
        
        return map1, map2
        
    def image_callback(self, msg):
        """Callback function for processing incoming images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
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
            
            # Publish uncompressed undistorted image (for web viewer)
            self.image_publisher.publish(undistorted_msg)
            
            # Create and publish compressed image
            self.publish_compressed_image(undistorted_image, msg.header)
            
            # Create and publish resized compressed image
            self.publish_resized_compressed_image(undistorted_image, msg.header)
            
            # Publish camera info for undistorted and resized images
            self.publish_camera_info(msg.header)
            
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
            
    def publish_resized_compressed_image(self, cv_image, header):
        """Publish resized and compressed JPEG image"""
        try:
            # Resize image
            resized_image = cv2.resize(cv_image, (self.resize_width, self.resize_height))
            
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
            
    def publish_camera_info(self, header):
        """Publish camera info for undistorted and resized images"""
        try:
            if self.undistorted_camera_matrix is None or self.resized_camera_matrix is None:
                return
                
            # Create camera info message for undistorted image
            undistorted_camera_info = CameraInfo()
            undistorted_camera_info.header = header
            undistorted_camera_info.width = self.image_size[0]
            undistorted_camera_info.height = self.image_size[1]
            
            # Camera matrix (K)
            undistorted_camera_info.k = [
                float(self.undistorted_camera_matrix[0, 0]), 0.0, float(self.undistorted_camera_matrix[0, 2]),
                0.0, float(self.undistorted_camera_matrix[1, 1]), float(self.undistorted_camera_matrix[1, 2]),
                0.0, 0.0, 1.0
            ]
            
            # Projection matrix (P) - same as K for rectified image
            undistorted_camera_info.p = [
                float(self.undistorted_camera_matrix[0, 0]), 0.0, float(self.undistorted_camera_matrix[0, 2]), 0.0,
                0.0, float(self.undistorted_camera_matrix[1, 1]), float(self.undistorted_camera_matrix[1, 2]), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Distortion coefficients (should be zero for undistorted image)
            undistorted_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            undistorted_camera_info.distortion_model = "plumb_bob"
            
            # Rectification matrix (identity for single camera)
            undistorted_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # Publish undistorted camera info
            self.undistorted_camera_info_publisher.publish(undistorted_camera_info)
            
            # Create camera info message for resized image
            resized_camera_info = CameraInfo()
            resized_camera_info.header = header
            resized_camera_info.width = self.resize_width
            resized_camera_info.height = self.resize_height
            
            # Camera matrix (K) for resized image
            resized_camera_info.k = [
                float(self.resized_camera_matrix[0, 0]), 0.0, float(self.resized_camera_matrix[0, 2]),
                0.0, float(self.resized_camera_matrix[1, 1]), float(self.resized_camera_matrix[1, 2]),
                0.0, 0.0, 1.0
            ]
            
            # Projection matrix (P) for resized image
            resized_camera_info.p = [
                float(self.resized_camera_matrix[0, 0]), 0.0, float(self.resized_camera_matrix[0, 2]), 0.0,
                0.0, float(self.resized_camera_matrix[1, 1]), float(self.resized_camera_matrix[1, 2]), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            # Distortion coefficients (should be zero for undistorted image)
            resized_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            resized_camera_info.distortion_model = "plumb_bob"
            
            # Rectification matrix (identity for single camera)
            resized_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # Publish resized camera info
            self.resized_camera_info_publisher.publish(resized_camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing camera info: {str(e)}')


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
