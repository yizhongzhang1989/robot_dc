import time  
import math
from math import radians  
import sys
import os
import subprocess
import numpy as np
import cv2
import json
from datetime import datetime

# Add the duco_robot_arm directory and its lib subdirectory to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
duco_robot_arm_dir = os.path.join(project_root, 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm')
sys.path.insert(0, duco_robot_arm_dir)
sys.path.insert(0, os.path.join(duco_robot_arm_dir, 'lib'))

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  

# Import KeypointTracker for keypoint tracking functionality
try:
    from estimation_task_keypoints_track import KeypointTracker
    KEYPOINT_TRACKER_AVAILABLE = True
except ImportError:
    print("Warning: estimation_task_keypoints_track module not found. Keypoint tracking will be skipped.")
    KEYPOINT_TRACKER_AVAILABLE = False

# Import CabinetFrameBuilder for cabinet frame establishment functionality
try:
    from estimation_task_build_cabinet_frame import CabinetFrameBuilder
    CABINET_FRAME_BUILDER_AVAILABLE = True
except ImportError:
    print("Warning: estimation_task_build_cabinet_frame module not found. Cabinet frame establishment will be skipped.")
    CABINET_FRAME_BUILDER_AVAILABLE = False

# Import functions from target pose estimation script
try:
    from estimation_task_find_target_pose import calculate_target_joint_angles, visualize_coordinate_systems, get_start_point_in_base
    TARGET_POSE_ESTIMATION_AVAILABLE = True
except ImportError:
    print("Warning: estimation_task_find_target_pose module not found. Target pose estimation will be skipped.")
    TARGET_POSE_ESTIMATION_AVAILABLE = False

# Robot connection parameters  
ip = '192.168.1.10'     # real robot
port = 7003  
      
def ConvertPose2Rad(pose):
    for idx, val in enumerate(pose):
        pose[idx] = math.radians(val)
    return pose

class IPCameraCapture:
    """IP camera screenshot class, manages connection and screenshot functionality"""
    
    def __init__(self, rtsp_url="rtsp://admin:123456@192.168.1.102/stream0"):
        self.rtsp_url = rtsp_url
        self.cap = None
        self.is_connected = False
        self.screenshot_dir = os.path.join(project_root, "temp/positioning_data")
        self.screenshot_counter = 1
        
        # Create save directory
        if not os.path.exists(self.screenshot_dir):
            os.makedirs(self.screenshot_dir)
    
    def connect(self):
        """connect to the IP camera"""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if self.cap.isOpened():
                # Set buffer size to minimize latency for IP cameras
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                self.is_connected = True
                print(f"Successfully connected to IP camera: {self.rtsp_url}")
                return True
            else:
                print(f"Failed to connect to IP camera: {self.rtsp_url}")
                return False
        except Exception as e:
            print(f"Error connecting to IP camera: {e}")
            return False
    
    def disconnect(self):
        """disconnect from the IP camera"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.is_connected = False
        print("IP camera connection has been disconnected")
    
    def get_robot_pose_data(self, robot):
        """Get current robot joint positions and TCP pose"""
        try:
            # Get joint positions
            joint_positions = robot.get_actual_joints_position()
            
            # Get TCP pose (end-effector pose)
            tcp_pose = robot.get_tcp_pose()
            
            return {
                "joint_positions": joint_positions,
                "tcp_pose": tcp_pose
            }
            
        except Exception as e:
            print(f"Error getting robot pose data: {e}")
            return None

    def get_latest_frame(self):
        """Get the latest frame by reconnecting to camera (ensures fresh frame)"""
        # Disconnect current connection
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        
        # Reconnect to get fresh stream
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if self.cap.isOpened():
                # Set buffer size to minimize latency
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                # Read a frame to ensure connection is stable
                ret, frame = self.cap.read()
                if ret:
                    print(f"Reconnected to camera successfully for frame capture")
                    return ret, frame
                else:
                    print("Failed to read frame after reconnection")
                    return False, None
            else:
                print("Failed to reconnect to camera")
                return False, None
        except Exception as e:
            print(f"Error reconnecting to camera: {e}")
            return False, None

    def capture_and_save(self, robot=None):
        """capture the current frame and save it with sequential numbering"""        
        try:
            # Get the latest frame by reconnecting (ensures fresh frame)
            ret, frame = self.get_latest_frame()
            
            if ret:
                # Generate filename with counter (like IP_camera_stream.py)
                filename = f"{self.screenshot_counter}.jpg"
                filepath = os.path.join(self.screenshot_dir, filename)
                
                # save the image
                success = cv2.imwrite(filepath, frame)
                if success:
                    print(f"Screenshot saved successfully: {filepath}")
                    print(f"Image size: {frame.shape[1]}x{frame.shape[0]}")
                    
                    # Get and save robot pose data if robot is provided
                    if robot is not None:
                        robot_data = self.get_robot_pose_data(robot)
                        if robot_data is not None:
                            # Create robot data filename
                            robot_filename = f"{self.screenshot_counter}.json"
                            robot_filepath = os.path.join(self.screenshot_dir, robot_filename)
                            
                            # Prepare data to save
                            pose_data = {
                                "screenshot_number": self.screenshot_counter,
                                "joint_positions": robot_data["joint_positions"],
                                "tcp_pose": robot_data["tcp_pose"],
                            }
                            
                            # Save robot pose data to JSON file
                            with open(robot_filepath, 'w') as f:
                                json.dump(pose_data, f, indent=2, ensure_ascii=False)
                                
                            print(f"Robot pose data saved: {robot_filepath}")
                            print(f"Joint positions: {robot_data['joint_positions']}")
                            print(f"TCP pose: {robot_data['tcp_pose']}")
                        else:
                            print("Warning: Could not get robot pose data")
                    
                    # Increment counter for next screenshot
                    self.screenshot_counter += 1
                    return True
                else:
                    print("Failed to save image")
                    return False
            else:
                print("Failed to read frame from camera")
                return False
                
        except Exception as e:
            print(f"Error capturing screenshot: {e}")
            return False

def main():  

    # Initialize IP camera connection
    print("Initializing IP camera connection...")
    camera = IPCameraCapture()
    if not camera.connect():
        print("Failed to initialize camera, exiting program")
        return

    # Create the DucoCobot instance and open connection  
    robot = DucoCobot(ip, port)  
    res = robot.open()  
    print("Open connection:", res)  
        
    # # Power on and enable the robot  
    # res = robot.power_on(True)  
    # print("Power on:", res)  
    # res = robot.enable(True)  
    # print("Enable:", res)  
        
    # Set up an Op instance with no triggering events (default)  
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
        
    # ====================================================================
    # # ==================================capture positioning data==============================           
    # start from near zero position
    print("Move to initial pose")
    initial_pose = [65.912,-25.975,75,46.301,-92.779,-111.484]
    ConvertPose2Rad(initial_pose)
    res = robot.movej2(initial_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    # move to a safe middle position
    middle_pose = [65.13, -24.23, -104.84, 46.77, -99.23, -111.14]
    print("Move to middle pose")
    ConvertPose2Rad(middle_pose)
    res = robot.movej2(middle_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    # move to a safe middle position
    middle_pose = [65.13, -24.23, -104.84, -43.0, -99.23, -111.14]
    print("Move to middle pose")
    ConvertPose2Rad(middle_pose)
    res = robot.movej2(middle_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    # move to a safe middle position
    print("Move to capture pose, start to capture images")
    capture_pose = [65.25, -38.05, -105.18, -42.96, -26.7, -86.01]   # deg
    ConvertPose2Rad(capture_pose)
    res = robot.movej2(capture_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    # move to capture position
    print("Move to capture pose, start to capture images")
    capture_pose = [33.22, -45.74, -100.69, -36.78, -58.63, -89.82]   # deg
    ConvertPose2Rad(capture_pose)
    res = robot.movej2(capture_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    # ================================capture 5 images=================================
    print("Start to capture...")

    # Note that tcp_move is based on the tool coordinate system, check the direction
    print("Capturing the 1/5 Image...")
    offset = [0,0,80/1000,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)
    res = camera.capture_and_save(robot)
    while not res:
        time.sleep(0.5)
    offset = [0,0,-80/1000,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)

    print("Capturing the 2/5 Image...")
    offset = [50/1000, 0,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)
    res = camera.capture_and_save(robot)
    while not res:
        time.sleep(0.5)
    offset = [-50/1000,0,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)

    print("Capturing the 3/5 Image...")
    offset = [0,50/1000,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)
    res = camera.capture_and_save(robot)
    while not res:
        time.sleep(0.5)
    offset = [0,-50/1000,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)

    print("Capturing the 4/5 Image...")
    offset = [-30/1000,50/1000,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)
    res = camera.capture_and_save(robot)
    while not res:
        time.sleep(0.5)
    offset = [30/1000,-50/1000,0,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)

    print("Capturing the 5/5 Image...")
    offset = [0,-40/1000,40/1000,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)
    res = camera.capture_and_save(robot)
    while not res:
        time.sleep(0.5)
    offset = [0,40/1000,-40/1000,0,0,0]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0,'', True, op)
    time.sleep(0.5)

    # move to a safe middle position
    print("Move to capture pose, start to capture images")
    capture_pose = [65.25, -38.05, -105.18, -42.96, -26.7, -86.01]   # deg
    ConvertPose2Rad(capture_pose)
    res = robot.movej2(capture_pose, 2.0, 1.0, 0.0, True, op)
    time.sleep(1.0)

    print("capture finish")

    # ==================================track keypoints in images==============================
    print("\nStarting keypoint tracking process...")
    
    # Check if KeypointTracker is available and run tracking
    if KEYPOINT_TRACKER_AVAILABLE:
        try:
            # Initialize the keypoint tracker
            api_url = "http://10.172.151.12:8009"  # Default API URL
            tracker = KeypointTracker(api_url=api_url)
            
            # Run the complete keypoint tracking pipeline
            print("Running keypoint tracking pipeline...")
            tracking_success = tracker.run_tracking_pipeline(bidirectional=False)
            
            if tracking_success:
                print("✅ Keypoint tracking completed successfully!")
            else:
                print("❌ Keypoint tracking failed!")
                
        except Exception as e:
            print(f"Error during keypoint tracking: {e}")
    else:
        print("❌ Keypoint tracking skipped - KeypointTracker not available")
    
    # ==================================3d positioning==============================
    print("\nStarting 3D coordinate estimation...")
    
    try:
        # Construct the command for 3D coordinate estimation
        estimation_command = [
            "python3", 
            "scripts/ThirdParty/robot_vision/core/3d_coordinates_estimation.py",
            "--reference-keypoints", "temp/positioning_data/ref_keypoints.json",
            "--reference-pose", "temp/positioning_data/ref_pose.json",
            "--keypoint-files", 
            "temp/positioning_data/1_tracking_result.json",
            "temp/positioning_data/2_tracking_result.json", 
            "temp/positioning_data/3_tracking_result.json",
            "temp/positioning_data/4_tracking_result.json",
            "temp/positioning_data/5_tracking_result.json",
            "--pose-files",
            "temp/positioning_data/1.json",
            "temp/positioning_data/2.json",
            "temp/positioning_data/3.json", 
            "temp/positioning_data/4.json",
            "temp/positioning_data/5.json",
            "--intrinsic-file", "temp/camera_parameters/calibration_result.json",
            "--extrinsic-file", "temp/camera_parameters/eye_in_hand_result.json",
            "--output", "./temp/3d_coordinate_estimation_result"
        ]
        
        # Change to project root directory and run the estimation
        project_root_dir = os.path.dirname(script_dir)
        subprocess.run(estimation_command, cwd=project_root_dir, 
                      capture_output=True, text=True, check=True)
        
        print("✅ 3D coordinate estimation completed successfully!")
        print("📁 Results saved to: temp/3d_coordinate_estimation_result/")
        
        # Load and display the results
        results_file = os.path.join(project_root_dir, "temp/3d_coordinate_estimation_result/3d_coordinate_estimation_result.json")
        if os.path.exists(results_file):
            with open(results_file, 'r') as f:
                estimation_results = json.load(f)
            
            print(f"📊 Successfully estimated 3D coordinates for {estimation_results['num_keypoints']} keypoints")
            print("🎯 3D Coordinates:")
            for keypoint in estimation_results['keypoints_3d']:
                coords = keypoint['coordinates_3d']
                print(f"   {keypoint['name']}: ({coords['x']:.3f}, {coords['y']:.3f}, {coords['z']:.3f}) m")
        
    except subprocess.CalledProcessError as e:
        print(f"❌ 3D coordinate estimation failed with error: {e}")
        print(f"Error output: {e.stderr}")
    except FileNotFoundError:
        print("❌ Required files not found for 3D coordinate estimation")
    except Exception as e:
        print(f"❌ Error during 3D coordinate estimation: {e}")
    
    # ==================================building frame==============================
    print("\nStarting cabinet frame coordinate system establishment...")
    
    # Check if CabinetFrameBuilder is available and run cabinet frame establishment
    if CABINET_FRAME_BUILDER_AVAILABLE:
        try:
            # Initialize the cabinet frame builder
            builder = CabinetFrameBuilder()
            
            # Check if 3D positioning results file exists
            project_root_dir = os.path.dirname(script_dir)
            positioning_results_file = os.path.join(project_root_dir, "temp/3d_coordinate_estimation_result/3d_coordinate_estimation_result.json")
            
            if not os.path.exists(positioning_results_file):
                print("❌ Missing required 3D positioning results file:")
                print(f"   - {positioning_results_file}")
                print("Please ensure 3D coordinate estimation completed successfully.")
            else:
                # Load 3D positioning results
                print("� Loading 3D positioning results...")
                positioning_data = builder.load_3d_positioning_results()
                
                print(f"✅ Loaded 3D positioning results with {len(builder.keypoints_3d)} keypoints")
                
                # Build cabinet coordinate system
                print("🏗️ Building cabinet coordinate system...")
                # Use point2 (ID=2) and point8 (ID=8) as in the original script
                available_ids = [kp['id'] for kp in builder.keypoints_3d]
                
                point2_id = 2
                point8_id = 8
                
                if point2_id not in available_ids:
                    print(f"   Warning: Point ID {point2_id} not found, using first available point instead")
                    point2_id = available_ids[0]
                
                if point8_id not in available_ids:
                    print(f"   Warning: Point ID {point8_id} not found, using second available point instead")
                    if len(available_ids) > 1:
                        point8_id = available_ids[1]
                    else:
                        raise ValueError(f"Need at least 2 keypoints to build coordinate system, but only {len(available_ids)} available")
                
                results = builder.create_cabinet_frame_results(point2_id=point2_id, point8_id=point8_id)
                
                # Save results
                print("💾 Saving results...")
                builder.save_results(results, "build_cabinet_frame_result.json")
                
                print("✅ Cabinet frame coordinate system established successfully!")
                print(f"📁 Results saved to: {builder.output_dir}")
                
                # Display the results
                print("🏗️ Cabinet coordinate system details:")
                if 'cabinet_pose' in results:
                    pose = results['cabinet_pose']
                    print(f"   Origin: ({pose['origin']['x']:.3f}, {pose['origin']['y']:.3f}, {pose['origin']['z']:.3f}) m")
                    print(f"   X-axis: [{pose['x_axis'][0]:.3f}, {pose['x_axis'][1]:.3f}, {pose['x_axis'][2]:.3f}]")
                    print(f"   Y-axis: [{pose['y_axis'][0]:.3f}, {pose['y_axis'][1]:.3f}, {pose['y_axis'][2]:.3f}]")
                    print(f"   Z-axis: [{pose['z_axis'][0]:.3f}, {pose['z_axis'][1]:.3f}, {pose['z_axis'][2]:.3f}]")
                
        except Exception as e:
            print(f"❌ Error during cabinet frame establishment: {e}")
    else:
        print("❌ Cabinet frame establishment skipped - CabinetFrameBuilder not available")
    
    # ===========================find target pose and move==============================
    print("\nStarting target pose estimation and robot movement...")
    
    # Check if target pose estimation is available
    if TARGET_POSE_ESTIMATION_AVAILABLE:
        try:
            # Step 1: Check if required files exist
            project_root_dir = os.path.dirname(script_dir)
            cabinet_frame_file = os.path.join(project_root_dir, "temp/build_cabinet_frame_result/build_cabinet_frame_result.json")
            target_pose_file = os.path.join(project_root_dir, "temp/positioning_data/target_pose.json")
            
            missing_files = []
            if not os.path.exists(cabinet_frame_file):
                missing_files.append(cabinet_frame_file)
            if not os.path.exists(target_pose_file):
                missing_files.append(target_pose_file)
                
            if missing_files:
                print("❌ Missing required files for target pose estimation:")
                for file in missing_files:
                    print(f"   - {file}")
                print("Please ensure cabinet frame establishment and target pose definition are completed.")
            else:
                print("📍 Calculating target joint angles using inverse kinematics...")
                
                # Step 2: Get target position and orientation in base coordinates
                target_xyz_m, target_rpy_deg = get_start_point_in_base()
                target_rpy_rad = [math.radians(a) for a in target_rpy_deg]
                target_pose = target_xyz_m + target_rpy_rad
                
                print(f"🎯 Target TCP position: ({target_xyz_m[0]:.3f}, {target_xyz_m[1]:.3f}, {target_xyz_m[2]:.3f}) m")
                print(f"🎯 Target TCP orientation: ({target_rpy_deg[0]:.1f}, {target_rpy_deg[1]:.1f}, {target_rpy_deg[2]:.1f}) deg")
                
                # Step 3: Calculate target joint angles using inverse kinematics
                target_joint_angles = calculate_target_joint_angles(robot)
                
                if target_joint_angles is not None:
                    target_joint_deg = [math.degrees(angle) for angle in target_joint_angles]
                    print("✅ Target joint angles calculated successfully!")
                    print(f"🔧 Target joint angles (deg): [{target_joint_deg[0]:.1f}, {target_joint_deg[1]:.1f}, {target_joint_deg[2]:.1f}, {target_joint_deg[3]:.1f}, {target_joint_deg[4]:.1f}, {target_joint_deg[5]:.1f}]")
                    
                    # Save target joint angles to JSON file
                    try:
                        # Create output directory if it doesn't exist
                        output_dir = os.path.join(project_root_dir, "temp/find_target_result")
                        os.makedirs(output_dir, exist_ok=True)
                        
                        # Prepare data to save
                        target_joint_data = {
                            "target_joint_angles_rad": [float(angle) for angle in target_joint_angles],
                            "target_joint_angles_deg": [float(angle_deg) for angle_deg in target_joint_deg],
                            "timestamp": datetime.now().isoformat(),
                            "description": "Target joint angles calculated by inverse kinematics from duco_auto_positioning.py"
                        }
                        
                        # Save to JSON file
                        output_file = os.path.join(output_dir, "target_joint_angles.json")
                        with open(output_file, 'w') as f:
                            json.dump(target_joint_data, f, indent=2, ensure_ascii=False)
                        
                        print(f"💾 Target joint angles saved to: {output_file}")
                    except Exception as save_error:
                        print(f"⚠️  Failed to save target joint angles: {save_error}")
                    
                    # # Step 4:  Move robot to target position
                    # # Uncomment the following lines if you want the robot to actually move
                    # print("🤖 Moving robot to target position...")
                    # res = robot.movej2(target_joint_angles, 1.0, 1.0, 0.0, True, op)
                    # if res:
                    #     print("✅ Robot moved to target position successfully!")
                    #     time.sleep(1.0)  # Wait for movement to complete
                    # else:
                    #     print("❌ Failed to move robot to target position")
                    
                    # Step 5: Visualize coordinate systems and save result
                    print("📊 Generating visualization...")
                    try:
                        visualize_coordinate_systems(robot, target_joint_angles)
                        print("✅ Visualization saved to: temp/find_target_result/find_target_results.jpg")
                    except Exception as viz_error:
                        print(f"⚠️  Visualization failed: {viz_error}")
                        
                else:
                    print("❌ Failed to calculate target joint angles - inverse kinematics solution not found")
                    
        except Exception as e:
            print(f"❌ Error during target pose estimation: {e}")
    else:
        print("❌ Target pose estimation skipped - required modules not available")
    
    # ====================================================================
    # disconnect the camera and robot arm
    camera.disconnect()
    
    # # Clean up: disable, power off, and close connection  
    # res = robot.disable(True)  
    # print("\nDisable result:", res)  
    # res = robot.power_off(True)  
    # print("Power off result:", res)  
    res = robot.close()  
    print("Close connection result:", res)  
     
if __name__ == '__main__':  
    try:  
        main()  
    except Thrift.TException as tx:  
        print("Thrift Exception:", tx.message)
    except Exception as e:
        print(f"Error: {e}")
