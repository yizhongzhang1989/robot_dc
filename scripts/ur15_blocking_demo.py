#!/usr/bin/env python3
"""
UR15 Blocking Motion Demo
Demonstrates the new blocking motion functions with safe callbacks.
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

from ur15 import UR15Robot, safe_callback


def demo_callbacks():
    """Demo callback functions - these will be made crash-proof automatically"""
    
    def on_movement_start(target):
        """Called when movement starts"""
        if len(target) == 6:  # Joint positions
            print(f"📍 Movement started to joints: {[round(j*180/math.pi, 1) for j in target]}°")
        else:  # TCP pose
            pos = target[:3]
            ori = target[3:]
            print(f"📍 Movement started to TCP: pos={[round(p*1000, 1) for p in pos]}mm, ori={[round(o*180/math.pi, 1) for o in ori]}°")
    
    def on_movement_progress(current, target, distance):
        """Called during movement (every 100ms)"""
        # Only print occasionally to avoid spam
        if hasattr(on_movement_progress, 'counter'):
            on_movement_progress.counter += 1
        else:
            on_movement_progress.counter = 0
        
        if on_movement_progress.counter % 10 == 0:  # Print every 1 second
            print(f"🔄 Progress: distance={distance:.4f}")
    
    def on_movement_done(target, final):
        """Called when movement completes successfully"""
        print(f"🎯 Movement completed successfully!")
        if len(target) == 6:  # Joint positions
            print(f"   Final joints: {[round(j*180/math.pi, 1) for j in final]}°")
        else:  # TCP pose
            pos = final[:3]
            print(f"   Final TCP position: {[round(p*1000, 1) for p in pos]}mm")
    
    def on_movement_error(error_msg):
        """Called when movement fails"""
        print(f"💥 Movement failed: {error_msg}")
        # This might have risky operations but won't crash the system
        risky_operation = 1 / 0  # This will print warning instead of crashing
    
    return on_movement_start, on_movement_progress, on_movement_done, on_movement_error


def main():
    """Main demo function"""
    print("="*60)
    print("🤖 UR15 Blocking Motion Demo")
    print("="*60)
    
    # Connect to robot
    robot = UR15Robot("192.168.1.15", 30002)
    if robot.open() != 0:
        print("❌ Failed to connect to robot")
        return
    
    try:
        # Get callback functions
        on_start, on_progress, on_done, on_error = demo_callbacks()
        
        print("\n1️⃣ Testing Blocking MoveJ with Callbacks")
        print("-" * 40)
        
        # Home position
        home_joints = [0, -1.57, 0, -1.57, 0, 0]
        
        success = robot.movej_blocking(
            q=home_joints,
            a=0.8,
            v=1.05,
            timeout=30,
            threshold=0.01,
            on_start=on_start,        # Safe - won't crash system
            on_progress=on_progress,  # Safe - won't crash system  
            on_done=on_done,          # Safe - won't crash system
            on_error=on_error         # Safe - even with risky operations
        )
        
        print(f"MoveJ Result: {'✅ Success' if success else '❌ Failed'}")
        
        if success:
            print("\n2️⃣ Testing Blocking MoveL with Callbacks")
            print("-" * 40)
            
            # Get current pose and move slightly
            current_pose = robot.get_actual_tcp_pose()
            if current_pose:
                target_pose = current_pose.copy()
                target_pose[2] += 0.05  # Move 5cm up in Z
                
                success = robot.movel_blocking(
                    pose=target_pose,
                    a=0.2,
                    v=0.1,  
                    timeout=30,
                    threshold=0.01,
                    on_start=on_start,
                    on_progress=on_progress,
                    on_done=on_done,
                    on_error=on_error
                )
                
                print(f"MoveL Result: {'✅ Success' if success else '❌ Failed'}")
        
        print("\n3️⃣ Testing Extensible Blocking Support")
        print("-" * 40)
        
        # Add blocking support to servoj (just as an example)
        robot.add_blocking_support("servoj", "joint")
        
        # Now you can use servoj_blocking()
        if hasattr(robot, 'servoj_blocking'):
            print("✅ servoj_blocking() method added successfully")
            print("   You can now use: robot.servoj_blocking(joints, timeout=30, on_done=callback)")
        
        print("\n4️⃣ Demonstrating Error Resistance")
        print("-" * 40)
        
        @safe_callback
        def risky_callback():
            """This callback will fail but won't crash the system"""
            print("Doing something risky...")
            result = 1 / 0  # Division by zero
            return result
        
        # Call the risky callback - it will print warning instead of crashing
        risky_callback()
        print("✅ System continued running after callback error!")
        
    except Exception as e:
        print(f"❌ Demo error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        robot.close()
        print("\n" + "="*60)
        print("🏁 Demo completed")
        print("="*60)


def usage_examples():
    """Show usage examples"""
    print("""
    
📖 USAGE EXAMPLES:

1. Basic Blocking Movement:
   success = robot.movej_blocking([0, -1.57, 0, -1.57, 0, 0])
   
2. With Callbacks:
   success = robot.movej_blocking(
       q=[0, -1.57, 0, -1.57, 0, 0],
       timeout=30,
       on_done=lambda target, final: print("Movement done!"),
       on_error=lambda msg: print(f"Error: {msg}")
   )

3. Add Blocking to Any Function:
   robot.add_blocking_support("servoj", "joint")
   success = robot.servoj_blocking(joints, timeout=30)

4. Safe Callbacks (Never Crash):
   @safe_callback
   def my_callback():
       return 1 / 0  # Prints warning instead of crashing
       
Key Benefits:
✅ Blocking operations with position monitoring
✅ Crash-proof callbacks that print warnings
✅ Extensible to any motion function
✅ Progress monitoring and error handling
✅ System continues running even with callback errors
    """)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--examples":
        usage_examples()
    else:
        main()