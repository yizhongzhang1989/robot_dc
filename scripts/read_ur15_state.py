#!/usr/bin/env python3
"""
Read real-time UR15 robot joint angles and TCP coordinates using the UR15Robot class.
This uses the existing UR15 control connection (port 30002) to get robot state.
"""

import sys
import os
import time
import math

# Add the ur15_robot_arm module to path
current_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(current_dir, '..'))
ur15_path = os.path.join(repo_root, 'colcon_ws/src/ur15_robot_arm/ur15_robot_arm')
sys.path.append(ur15_path)

from ur15 import UR15Robot


def display_robot_state(joint_positions, tcp_pose):
    """Display formatted robot state information."""
    # Convert joint angles to degrees
    joint_degrees = [math.degrees(pos) for pos in joint_positions] if joint_positions else [0]*6
    
    # Clear screen (optional - comment out if not wanted)
    os.system('cls' if os.name == 'nt' else 'clear')
    
    print("🤖 UR15 REAL-TIME ROBOT STATE")
    print("="*60)
    
    # Joint angles
    print("📐 JOINT ANGLES:")
    for i, angle in enumerate(joint_degrees):
        print(f"   Joint {i+1}: {angle:8.2f}°")
    
    print("\n📍 TCP CARTESIAN COORDINATES:")
    if tcp_pose and len(tcp_pose) >= 6:
        # Position in mm for better readability  
        print(f"   Position X: {tcp_pose[0]*1000:8.2f} mm")
        print(f"   Position Y: {tcp_pose[1]*1000:8.2f} mm") 
        print(f"   Position Z: {tcp_pose[2]*1000:8.2f} mm")
        print(f"   Rotation Rx: {math.degrees(tcp_pose[3]):8.2f}°")
        print(f"   Rotation Ry: {math.degrees(tcp_pose[4]):8.2f}°") 
        print(f"   Rotation Rz: {math.degrees(tcp_pose[5]):8.2f}°")
    else:
        print("   No TCP data available")
    
    print("="*60)
    print(f"📡 Data updated at: {time.strftime('%H:%M:%S')}")
    print("\n⏹️  Press Ctrl+C to stop")


def main():
    """Main function to monitor UR15 robot state."""
    print("🚀 UR15 Robot State Monitor")
    print("📋 This reads real-time joint angles and TCP coordinates from UR15")
    print()
    
    # UR15 connection parameters (from your ur_test.py)
    ur15_ip = "192.168.1.15"
    ur15_port = 30002
    
    # Create robot instance
    robot = UR15Robot(ur15_ip, ur15_port)
    
    try:
        # Connect to robot
        print(f"🔗 Connecting to UR15 at {ur15_ip}:{ur15_port}...")
        if robot.open() != 0:
            print("❌ Failed to connect to UR15 robot")
            return
        
        print(f"✅ Connected to UR15 robot")
        print("📡 Starting real-time monitoring...")
        print("⏹️  Press Ctrl+C to stop")
        time.sleep(2)
        
        # Main monitoring loop
        while True:
            # Get current joint positions
            joint_positions = robot.get_actual_joint_positions()
            
            # Get current TCP pose
            tcp_pose = robot.get_actual_tcp_pose()
            
            # Display the data
            display_robot_state(joint_positions, tcp_pose)
            
            # Update rate (10 Hz like the original)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n⏹️  Stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Close connection
        try:
            robot.close()
            print("🔌 Disconnected from UR15 robot")
        except:
            pass


if __name__ == '__main__':
    main()