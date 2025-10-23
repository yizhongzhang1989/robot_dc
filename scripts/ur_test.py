from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import math

def tcp_normalize(robot, a=0.5, v=0.2):
    """
    Normalize TCP orientation so that:
    - TCP z+ aligns with base z- (pointing downward)
    - TCP y+ aligns with base y+ (same direction)
    - TCP x+ aligns with base x- (opposite direction)
    """
    # Get current TCP pose
    current_pose = robot.get_actual_tcp_pose()
    if current_pose is None:
        print("[ERROR] Failed to get current TCP pose")
        return -1
    
    # Extract position (keep the same)
    x, y, z = current_pose[0], current_pose[1], current_pose[2]
    
    # Set orientation: rotation of pi around Y axis
    # This gives: X_tcp = -X_base, Y_tcp = Y_base, Z_tcp = -Z_base
    rx, ry, rz = 0.0, math.pi, 0.0
    
    # Create target pose
    target_pose = [x, y, z, rx, ry, rz]
    
    print(f"Current TCP pose: {current_pose}")
    print(f"Target normalized pose: {target_pose}")
    
    # Move to target pose using movel (linear movement in base frame)
    result = robot.movel(target_pose, a=a, v=v)
    
    if result == 0:
        print("[SUCCESS] TCP normalized successfully")
    else:
        print("[ERROR] Failed to normalize TCP")
    
    return result

def main():
    # Definition of robot IP and port
    ur15_ip = "192.168.1.15"
    ur15_port = 30002
    rs485_port = 54321

    # Init the robot object
    robot = UR15Robot(ur15_ip, ur15_port)
    # Open connection of robot
    ur15_start_res = robot.open()

    # Open RS485 socket connection
    rs485_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rs485_start_res = rs485_socket.connect((ur15_ip, rs485_port))
    print(f"RS485 socket connection result: {rs485_start_res}")

    # =====================================Program===========================
    if ur15_start_res == 0:

        # # 1. Test popup function
        # popup_res = robot.popup("Test Popup1", title="Test Title", warning=True)
        # print(f"Popup result: {popup_res}")

        # # 2. move to zero position
        # res = robot.movej([0, -1.57, 0, -1.57, 0, 0], a=0.8, v=1.05)
        # print(f"MoveJ result: {res}")
        # time.sleep(1)

        # 3. enter freedrive mode f
        freedrive_duration = 20 #seconds
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

        # # 5. get actual TCP pose and joint positions
        # actual_pose = robot.get_actual_tcp_pose()
        # actual_joints = robot.get_actual_joint_positions()

        # if actual_pose:
        #     print(f"Actual tcp pose (m, rad): {actual_pose}")
        # if actual_joints:
        #     print(f"Actual joint positions (rad): {actual_joints}")
        #     print(f"Actual joint positions (deg): {[j*180/3.1415926 for j in actual_joints]}")

        # # 6. move tcp
        # res = robot.move_tcp([0, 0, 0.1, 0, 0, 0], a=0.5, v=0.2)
        # print(f"Move TCP result: {res}")

        # time.sleep(1)
        # # 6.1 Test tcp_normalize function
        # res = tcp_normalize(robot, a=0.2, v=0.2)
        # print(f"TCP normalize result: {res}")

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
        
        # # 8. set tool voltage
        # res = robot.set_tool_voltage(24)  # Set to 24v
        # print(f"Set tool voltage result: {res}")
        # time.sleep(30)

        # res = robot.set_tool_voltage(0)   # Set to 0v
        # print(f"Set tool voltage result: {res}")
        # time.sleep(1)  

        # # 9. based on RS485.urcap to Send lock command but can not receive response yet
        lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
        unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
        status_command = [0x53, 0x26, 0x02, 0x01, 0x01, 0xCA, 0xD4]
        # # res = robot.socket_open('192.168.1.15',54321)
        # # res = robot.socket_send_all('192.168.1.15', 54321, lock_command, socket_name='socket_0')
        # # print(f"RS485 Lock command result: {res}")

        # # or Directly use socket to send and receive data
        # rs485_socket.sendall(bytes(status_command))
        # time.sleep(0.5)
        # rs485_data = rs485_socket.recv(1024)
        # print('Rs485 Received Data:', ' '.join(f'0x{b:02x}' for b in rs485_data))
        # time.sleep(0.5)
        # rs485_socket.sendall(bytes(lock_command))
        # time.sleep(0.5)
        # rs485_data = rs485_socket.recv(1024)
        # print('Rs485 Received Data:', ' '.join(f'0x{b:02x}' for b in rs485_data))
        # time.sleep(0.5)
        # rs485_socket.sendall(bytes(status_command))
        # time.sleep(0.5)
        # rs485_data = rs485_socket.recv(1024)
        # print('Rs485 Received Data:', ' '.join(f'0x{b:02x}' for b in rs485_data))
        # time.sleep(0.5)
        

    else:
        print(f"Failed to connect to robot at {ur15_ip}:{ur15_port}")
    

    rs485_socket.close()

    # 10. Power down and close connection of the robot   
    # res = robot.powerdown()
    # print(f"Power down result: {res}")
    
    # 11. Close connection
    close_res = robot.close()
    print(f"Close result: {close_res}")

if __name__ == "__main__":
    main()