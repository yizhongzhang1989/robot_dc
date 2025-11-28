#!/usr/bin/env python3
"""
Image Capture Handler
"""

import os
import cv2
import time
import rclpy
from typing import Dict, Any, Optional
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ur15_workflow.base import OperationHandler


class CaptureImageHandler(OperationHandler):
    """
    Handler for image capture operations
    """
    
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
    
    def image_callback(self, msg):
        """Callback for image subscription"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            print(f"Error converting image: {e}")

    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Capture image from camera
        
        Operation parameters:
            - camera_topic: ROS topic for camera (default: /ur15_camera/image_raw)
            - save_path: Where to save image
            - timeout: Timeout for image capture (default: 5.0)
        """
        subscription = None
        temp_node = False
        
        try:
            print("    Capturing image...")
            
            # Get parameters
            camera_topic = operation.get('camera_topic', context.get('camera_topic', '/ur15_camera/image_raw'))
            save_path = self._resolve_parameter(operation.get('save_path'), context)
            timeout = operation.get('timeout', 5.0)
            
            if not save_path:
                return {'status': 'error', 'error': 'No save_path specified'}
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            
            # Get ROS node
            node = context.get('ros_node')
            if node is None:
                # Create temporary node if not in context
                if not rclpy.ok():
                    rclpy.init()
                node = rclpy.create_node('capture_handler_temp_node')
                temp_node = True
            
            # Reset state
            self.latest_image = None
            self.image_received = False
            
            # Subscribe
            print(f"    Subscribing to {camera_topic}...")
            subscription = node.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                1
            )
            
            # Wait for image
            start_time = time.time()
            while not self.image_received:
                if time.time() - start_time > timeout:
                    return {'status': 'error', 'error': f'Timeout waiting for image on {camera_topic}'}
                
                rclpy.spin_once(node, timeout_sec=0.1)
            
            # Save image
            if self.latest_image is not None:
                cv2.imwrite(save_path, self.latest_image)
                print(f"    ✓ Image saved to {save_path}")
                
                return {
                    'status': 'success',
                    'outputs': {
                        'captured_image_path': save_path
                    }
                }
            else:
                return {'status': 'error', 'error': 'Image received but is None'}
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
            
        finally:
            # Cleanup
            if subscription is not None and node is not None:
                node.destroy_subscription(subscription)
            
            if temp_node and node is not None:
                node.destroy_node()
