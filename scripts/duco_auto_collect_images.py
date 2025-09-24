#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import os
import math
import numpy as np
import json
import cv2
import threading
import psutil  # For system monitoring

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# get current file path
current_file_path = os.path.dirname(os.path.abspath(__file__))
repo_root_path = os.path.abspath(os.path.join(current_file_path, '..'))
duco_script_path = os.path.join(repo_root_path, 'colcon_ws/src/duco_robot_arm/duco_robot_arm')
gen_py_path = os.path.join(duco_script_path, 'gen_py')
lib_path = os.path.join(current_file_path, 'lib')
# Add the required paths for thrift and generated code
sys.path.append(duco_script_path)
sys.path.append(gen_py_path)
sys.path.append(lib_path)

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
from duco_FTCApiPost import *

ip_robot = '192.168.1.10'
port_robot = 7003

IMAGE_TOPIC = "/robot_arm_camera/image_raw"

class ROS2ImageSubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('image_subscriber_node')
        self.topic_name = topic_name
        self.frame = None
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.subscription = None
        self.running = False
        
    def start(self):
        """start subscribing to the image topic"""
        if self.running:
            return True
            
        try:
            print(f"Subscribing to topic: {self.topic_name}")
            
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,  # Allow frame drops - no blocking!
                durability=DurabilityPolicy.VOLATILE,       # Don't store messages
                history=HistoryPolicy.KEEP_LAST,           # Only keep latest messages
                depth=1                                     # Only keep the most recent frame
            )
            
            self.subscription = self.create_subscription(
                Image,
                self.topic_name,
                self.image_callback,
                qos_profile  # Use optimized QoS instead of default
            )
            self.running = True
            
            # Wait for the first frame
            print("Waiting for first image...")
            for _ in range(50):  # Wait up to 5 seconds
                if self.frame is not None:
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.frame is not None:
                print("Image topic connected successfully")
                return True
            else:
                print("Failed to receive first image")
                self.stop()
                return False
                
        except Exception as e:
            self.running = False
            print(f"Failed to start image subscriber: {e}")
            return False

    def image_callback(self, msg):
        """callback function for image topic"""
        try:
            # Skip processing if we already have a recent frame (reduce CPU usage)
            with self.lock:
                if self.frame is not None:
                    # Only update every few frames to reduce CPU load
                    import time
                    if not hasattr(self, '_last_update_time'):
                        self._last_update_time = 0
                    current_time = time.time()
                    if current_time - self._last_update_time < 0.1:  # Limit to 10 FPS for this script
                        return
                    self._last_update_time = current_time
            
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.frame = cv_image.copy()
        except Exception as e:
            print(f"Error converting image: {e}")

    def get_frame(self):
        """get the current frame"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """stop subscribing to the image topic"""
        if not self.running:
            return
            
        self.running = False
        if self.subscription:
            self.destroy_subscription(self.subscription)
        with self.lock:
            self.frame = None
        print("Image subscriber stopped")

def ensure_auto_collect_dir():
    """ensure the auto collect directory exists"""
    auto_collect_dir = "/home/a/Documents/robot_dc2/temp/camera_calibration_data"
    if not os.path.exists(auto_collect_dir):
        os.makedirs(auto_collect_dir)
        print(f"Created directory: {auto_collect_dir}")
    return auto_collect_dir

def compute_transform_matrix(x, y, z, rx, ry, rz):
    """calculate the 4x4 transformation matrix from position and orientation"""
    try:
        # calculate rotation matrix R = Rz(rz) * Ry(ry) * Rx(rx)
        cos_rx, sin_rx = math.cos(rx), math.sin(rx)
        cos_ry, sin_ry = math.cos(ry), math.sin(ry)
        cos_rz, sin_rz = math.cos(rz), math.sin(rz)

        # Rx rotation matrix
        Rx = np.array([
            [1, 0, 0],
            [0, cos_rx, -sin_rx],
            [0, sin_rx, cos_rx]
        ])
        
        # Ry rotation matrix
        Ry = np.array([
            [cos_ry, 0, sin_ry],
            [0, 1, 0],
            [-sin_ry, 0, cos_ry]
        ])
        
        # Rz rotation matrix
        Rz = np.array([
            [cos_rz, -sin_rz, 0],
            [sin_rz, cos_rz, 0],
            [0, 0, 1]
        ])
        
        # R = Rz * Ry * Rx
        R = np.dot(np.dot(Rz, Ry), Rx)
        
        # Construct 4x4 transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 0:3] = R
        transform_matrix[0:3, 3] = [x, y, z]

        # Convert to Python list format to maintain consistency with original json file format
        return transform_matrix.tolist()
        
    except Exception as e:
        print(f"Error computing transform matrix: {e}")
        return None

def get_robot_pose(duco_cobot):
    """get the current robot pose including joint angles and end-effector pose"""
    try:
        # get joint angles (radians)
        joint_angles = duco_cobot.get_actual_joints_position()
        
        # get end-effector pose (x, y, z in meters; rx, ry, rz in radians)
        end_pose = duco_cobot.get_tcp_pose()
        
        # calculate 4x4 transformation matrix
        transform_matrix = compute_transform_matrix(
            end_pose[0], end_pose[1], end_pose[2],  # x, y, z
            end_pose[3], end_pose[4], end_pose[5]   # rx, ry, rz
        )
        
        return {
            "joint_angles": joint_angles,
            "end_xyzrpy": {
                "x": end_pose[0],
                "y": end_pose[1], 
                "z": end_pose[2],
                "rx": end_pose[3],
                "ry": end_pose[4],
                "rz": end_pose[5]
            },
            "end2base": transform_matrix
        }
    except Exception as e:
        print(f"Error getting robot pose: {e}")
        return None

def save_pose_data(pose_data, image_index, save_dir):
    """save pose data to json file"""
    try:
        filename = f"{image_index}.json"
        filepath = os.path.join(save_dir, filename)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(pose_data, f, indent=2, ensure_ascii=False)
        
        print(f"Pose data saved: {filepath}")
        return True
    except Exception as e:
        print(f"Error saving pose data {image_index}: {e}")
        return False

def capture_image_and_pose(camera, duco_cobot, image_index, save_dir):
    """capture image and robot pose, save to files"""
    try:
        # get current robot pose
        pose_data = get_robot_pose(duco_cobot)
        if pose_data is None:
            print(f"Failed to get robot pose for image {image_index}")
            return False
        
        # capture image
        frame = camera.get_frame()
        if frame is not None:
            # save image
            img_filename = f"{image_index}.jpg"
            img_filepath = os.path.join(save_dir, img_filename)
            
            img_success = cv2.imwrite(img_filepath, frame)
            if img_success:
                print(f"Image saved: {img_filepath}")
            else:
                print(f"Failed to save image: {img_filepath}")
                return False
            
            # save pose data
            pose_success = save_pose_data(pose_data, image_index, save_dir)
            
            return img_success and pose_success
        else:
            print("No frame available from camera")
            return False
            
    except Exception as e:
        print(f"Error capturing image and pose {image_index}: {e}")
        return False

def collect_joint_angles_from_temp():
    """
    Read all JSON files from temp folder and extract joint angle information,
    save to collect_point.json file
    """
    temp_folder = "/home/a/Documents/robot_dc2/temp/auto_collect_points"
    output_file = os.path.join(temp_folder, "collect_points.json")
    
    collected_points = []
    
    # Get all numbered JSON files (exclude other special JSON files)
    json_files = []
    for i in range(100):  # assume max 100 files
        json_file = os.path.join(temp_folder, f"{i}.json")
        if os.path.exists(json_file):
            json_files.append(json_file)
    
    # Sort by filename to ensure correct order
    json_files.sort(key=lambda x: int(os.path.basename(x).split('.')[0]))
    
    print(f"Found {len(json_files)} JSON files")
    
    # Read each JSON file
    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # Extract joint_angles
            if 'joint_angles' in data:
                joint_angles = data['joint_angles']
                
                # Create point info record
                point_info = {
                    "point_id": int(os.path.basename(json_file).split('.')[0]),
                    "joint_angles": joint_angles
                }
                
                collected_points.append(point_info)
                print(f"Read file: {os.path.basename(json_file)}, Joint angles: {joint_angles}")
                
            else:
                print(f"Warning: 'joint_angles' field not found in {os.path.basename(json_file)}")
                
        except Exception as e:
            print(f"Error reading file {os.path.basename(json_file)}: {str(e)}")
    
    # Save result to collect_point.json
    result_data = {
        "total_points": len(collected_points),
        "points": collected_points
    }
    
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, indent=2, ensure_ascii=False)
        
        print(f"\nSuccessfully saved {len(collected_points)} data points to {output_file}")
        print(f"Total collected {len(collected_points)} joint angles")
        
    except Exception as e:
        print(f"Error saving file: {str(e)}")
    
    return collected_points

def print_collected_summary(collected_points):
    """
    Print summary of collected data
    """
    print("\n=== Data Collection Summary ===")
    print(f"Total data points: {len(collected_points)}")
    
    if collected_points:
        print(f"First point (ID: {collected_points[0]['point_id']}): {collected_points[0]['joint_angles']}")
        print(f"Last point (ID: {collected_points[-1]['point_id']}): {collected_points[-1]['joint_angles']}")

def convert_all_to_json():

    print("Starting to collect joint angles data from temp folder...")
    collected_data = collect_joint_angles_from_temp()
    print_collected_summary(collected_data)
    print("Data collection completed!")
    return 

def ConvertDeg2Rad(pose):

    result = []
    for val in pose:
        result.append(math.radians(val))
    return result

def load_collect_points():
    """
    Load joint angles from collect_points.json file
    """
    temp_folder = "/home/a/Documents/robot_dc2/temp"
    json_file = os.path.join(temp_folder, "collect_points.json")
    
    if not os.path.exists(json_file):
        print(f"Error: {json_file} not found!")
        return None
    
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        points = data.get('points', [])
        print(f"Loaded {len(points)} points from {json_file}")
        return points
    
    except Exception as e:
        print(f"Error loading points from {json_file}: {str(e)}")
        return None

def traverse_points(duco_cobot, points, op, camera=None, save_dir=None):
    """
    Traverse all points using movej2 command and capture images
    """
    if not points:
        print("No points to traverse!")
        return
    
    print(f"Starting to traverse {len(points)} points...")
    if camera and save_dir:
        print(f"Camera enabled - Images will be saved to: {save_dir}")
    
    for i, point in enumerate(points):
        point_id = point.get('point_id', i)
        joint_angles = point.get('joint_angles', [])
        
        if len(joint_angles) != 6:
            print(f"Warning: Point {point_id} has invalid joint angles (expected 6, got {len(joint_angles)})")
            continue
        
        print(f"\nMoving to Point {point_id} ({i+1}/{len(points)})")
        print(f"Joint angles (rad): {joint_angles}")
        
        try:
            # Execute movej2 command with joint angles in radians
            res = duco_cobot.movej2(joint_angles, 2.0, 1.0, 0.0, True, op)
            print(f"movej2 result: {res}")
            
            # Wait for movement to complete
            time.sleep(2.0)
            print(f"Successfully moved to Point {point_id}")
            
            # Capture image and pose after reaching the point
            if camera and save_dir:
                print(f"Capturing image and pose at Point {point_id}...")
                # Wait a bit more for stabilization
                time.sleep(1.0)
                
                # Multiple attempts to get fresh image without blocking web interface
                # Use shorter timeout and multiple spins to reduce blocking time
                for attempt in range(3):
                    rclpy.spin_once(camera, timeout_sec=0.05)  # Reduced timeout
                    if camera.get_frame() is not None:
                        break
                    time.sleep(0.1)  # Short delay between attempts
                
                success = capture_image_and_pose(camera, duco_cobot, i, save_dir)
                if success:
                    print(f"✓ Image {i}.jpg and pose {i}.json captured successfully")
                else:
                    print(f"✗ Failed to capture image and pose at Point {point_id}")
            
        except Exception as e:
            print(f"Error moving to Point {point_id}: {str(e)}")
            continue
    
    print(f"\nCompleted traversing all {len(points)} points!")
    if camera and save_dir:
        print(f"All images saved to: {save_dir}")

def main():
    camera = None
    
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Initialize DucoCobot instance
        duco_cobot = DucoCobot(ip_robot, port_robot)
        op = Op()
        op.time_or_dist_1 = 0
        op.trig_io_1 = 1
        op.trig_value_1 = False
        op.trig_time_1 = 0.0
        op.trig_dist_1 = 0.0
        op.trig_event_1 = ""
        op.time_or_dist_2 = 0
        op.trig_io_2 = 1
        op.trig_value_2 = False
        op.trig_time_2 = 0.0
        op.trig_dist_2 = 0.0
        op.trig_event_2 = ""

        # =======================initialize camera=======================
        print("Initializing ROS2 image subscriber...")
        
        # Lower process priority to reduce impact on web interface
        # This ensures web interface gets higher CPU priority
        try:
            import os
            os.nice(10)  # Lower priority (higher nice value)
            print("Process priority lowered to reduce web interface impact")
        except Exception as e:
            print(f"Warning: Could not lower process priority: {e}")
        
        camera = ROS2ImageSubscriber(IMAGE_TOPIC)
        camera_success = camera.start()
        
        if not camera_success:
            print("Warning: ROS2 image subscriber initialization failed, continuing without camera...")
            camera = None

        # =======================prepare save directory=======================
        save_dir = ensure_auto_collect_dir()
        
        # =======================open, power on and enable=======================
        rlt = duco_cobot.open()
        print("open:", rlt)
        # rlt = duco_cobot.power_on(True)
        # print("power_on:", rlt)
        # rlt = duco_cobot.enable(True)
        # print("enable:", rlt)
        
        # =====================================================================
        # Load collect points from JSON file
        points = load_collect_points()
        
        if points:
            # Traverse all points using movej2 command with camera
            traverse_points(duco_cobot, points, op, camera, save_dir)
        else:
            print("Failed to load points, please check...")

        # =======================disable, power off and close=======================
        # rlt = duco_cobot.disable(True)  
        # print("\nDisable result:", rlt)
        # rlt= duco_cobot.power_off(True)  
        # print("Power off result:", rlt)
        rlt = duco_cobot.close()
        print("close:", rlt)
        
    finally:
        # Clean up camera resources
        if camera:
            print("Stopping ROS2 image subscriber...")
            camera.stop()
            camera.destroy_node()
        
        # Shutdown ROS2
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
    