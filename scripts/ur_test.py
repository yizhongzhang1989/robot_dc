from ur15_robot_arm.ur15 import UR15Robot
import time

def main():
    ip = "192.168.1.15"
    port = 30002
    robot = UR15Robot(ip, port)
    
    # Open connection
    res = robot.open()
    
    if res == 0:
        
        # # 1. Test popup function
        # popup_res = robot.popup("Test Popup1", title="Test Title", warning=True)
        # print(f"Popup result: {popup_res}")

        # # 2. move to zero position
        # res = robot.movej([0, -1.57, 0, -1.57, 0, 0], a=0.8, v=1.05)
        # print(f"MoveJ result: {res}")
        # time.sleep(1)

        # 3. enter freedrive mode for 15 seconds
        freedrive_duration = 15 #seconds
        robot.freedrive_mode(duration=freedrive_duration)
        # wait for freedrive_duration seconds to end freedrive mode
        robot.end_freedrive_mode()

        # # 4. get target TCP pose and joint positions
        # pose = robot.get_target_tcp_pose()
        # joints = robot.get_target_joint_positions()

        # if pose:
        #     print(f"Target tcp pose (m, rad): {pose}")
        # if joints:
        #     print(f"Target joint positions (rad): {joints}")

        # 5. get actual TCP pose and joint positions
        actual_pose = robot.get_actual_tcp_pose()
        actual_joints = robot.get_actual_joint_positions()

        if actual_pose:
            print(f"Actual tcp pose (m, rad): {actual_pose}")
        if actual_joints:
            print(f"Actual joint positions (rad): {actual_joints}")

        # # 6. move tcp
        # res = robot.move_tcp([0, 0, 0.1, 0, 0, 0], a=0.5, v=0.2)
        # print(f"Move TCP result: {res}")

        # # 7. tcp force reading
        # try:
        #     while True:
        #         # Get force/torque data
        #         ft = robot.get_tcp_force()
                
        #         if ft:
        #             fx, fy, fz = ft[0], ft[1], ft[2]
        #             tx, ty, tz = ft[3], ft[4], ft[5]
                    
        #             # Calculate magnitude
        #             force_mag = (fx**2 + fy**2 + fz**2)**0.5
        #             torque_mag = (tx**2 + ty**2 + tz**2)**0.5
                    
        #             # Print on same line with \r (carriage return)
        #             print(f"\rFx:{fx:7.2f}N | Fy:{fy:7.2f}N | Fz:{fz:7.2f}N | "
        #                   f"Tx:{tx:7.3f}Nm | Ty:{ty:7.3f}Nm | Tz:{tz:7.3f}Nm | "
        #                   f"|F|:{force_mag:7.2f}N | |T|:{torque_mag:7.3f}Nm", end='', flush=True)
        #         else:
        #             print(f"\r[ERROR] Failed to read force/torque", end='', flush=True)
                
        #         time.sleep(0.1)
        
        # except KeyboardInterrupt:
        #     print("\n\nStopped by user (Ctrl+C)")
    else:
        print(f"Failed to connect to robot at {ip}:{port}")
        
    # res = robot.powerdown()
    # print(f"Power down result: {res}")
    
    # Close connection
    close_res = robot.close()
    print(f"Close result: {close_res}")

if __name__ == "__main__":
    main()