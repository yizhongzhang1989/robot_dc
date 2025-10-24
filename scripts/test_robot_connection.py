#!/usr/bin/env python3

from ur15_robot_arm.ur15 import UR15Robot
import time

def test_robot_connection():
    """Test UR15Robot connection and basic functionality"""
    
    try:
        # Initialize robot
        print("Initializing UR15Robot...")
        robot_ip = "192.168.1.15"
        robot_port = 30002
        robot = UR15Robot(ip=robot_ip, port=robot_port)
        print("✓ Robot initialized successfully")
        
        # Test connection status
        print("\nTesting connection status...")
        
        # Try to get current TCP pose
        print("Getting current TCP pose...")
        current_pose = robot.get_actual_tcp_pose()
        if current_pose is not None:
            print(f"✓ Current TCP pose: {[f'{p:.4f}' for p in current_pose]}")
        else:
            print("✗ Failed to get TCP pose")
        
        # Try to get joint positions
        print("Getting current joint positions...")
        joint_positions = robot.get_actual_joint_positions()
        if joint_positions is not None:
            print(f"✓ Joint positions: {[f'{j:.4f}' for j in joint_positions]}")
        else:
            print("✗ Failed to get joint positions")
        
        # Check robot mode (if available)
        print("Checking robot status...")
        
        # Test small movement (if robot is in correct mode)
        print("\nTesting small movement...")
        if current_pose is not None:
            # Create a small test movement (1mm in Z)
            test_pose = current_pose.copy()
            test_pose[2] += 0.001  # 1mm up
            
            print(f"Test pose: {[f'{p:.6f}' for p in test_pose]}")
            print("Attempting small movement...")
            
            result = robot.movel(test_pose, a=0.1, v=0.01)
            print(f"Movement result: {result}")
            
            if result == 0:
                print("✓ Movement successful")
                # Move back
                time.sleep(1)
                result = robot.movel(current_pose, a=0.1, v=0.01)
                print(f"Return movement result: {result}")
            else:
                print(f"✗ Movement failed with code: {result}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_robot_connection()