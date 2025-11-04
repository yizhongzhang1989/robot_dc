from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import math
import numpy as np


def rotvec_to_matrix(rx, ry, rz):
    """Convert rotation vector to rotation matrix"""
    angle = math.sqrt(rx**2 + ry**2 + rz**2)
    if angle < 1e-10:
        return np.eye(3)
    
    # Normalize axis
    kx, ky, kz = rx/angle, ry/angle, rz/angle
    
    # Rodrigues' rotation formula
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1 - c
    
    R = np.array([
        [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s],
        [ky*kx*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s],
        [kz*kx*v - ky*s, kz*ky*v + kx*s, kz*kz*v + c]
    ])
    return R


def matrix_to_rotvec(R):
    """Convert rotation matrix to rotation vector"""
    trace = np.trace(R)
    angle = math.acos(np.clip((trace - 1) / 2, -1.0, 1.0))
    
    if angle < 1e-10:
        return 0.0, 0.0, 0.0
    elif abs(angle - math.pi) < 1e-6:
        # 180-degree rotation
        if R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
            kx = math.sqrt((R[0, 0] + 1) / 2)
            ky = R[0, 1] / (2 * kx)
            kz = R[0, 2] / (2 * kx)
        elif R[1, 1] >= R[2, 2]:
            ky = math.sqrt((R[1, 1] + 1) / 2)
            kx = R[0, 1] / (2 * ky)
            kz = R[1, 2] / (2 * ky)
        else:
            kz = math.sqrt((R[2, 2] + 1) / 2)
            kx = R[0, 2] / (2 * kz)
            ky = R[1, 2] / (2 * kz)
    else:
        # General case
        kx = (R[2, 1] - R[1, 2]) / (2 * math.sin(angle))
        ky = (R[0, 2] - R[2, 0]) / (2 * math.sin(angle))
        kz = (R[1, 0] - R[0, 1]) / (2 * math.sin(angle))
    
    return angle * kx, angle * ky, angle * kz


def tool_orientation_correction(robot, z_rotation_deg=30, a=0.5, v=0.2):
    """
    Correct tool orientation by rotating around its Z-axis.
    """
    # Get current TCP pose
    current_pose = robot.get_actual_tcp_pose()
    if current_pose is None:
        print("[ERROR] Failed to get current TCP pose")
        return -1
    
    # Extract position and orientation
    x, y, z = current_pose[0], current_pose[1], current_pose[2]
    rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
    
    # Get current rotation matrix
    R_current = rotvec_to_matrix(rx, ry, rz)
    
    # Create rotation matrix for Z-axis rotation (in tool frame)
    # Rotation around Z-axis
    angle_rad = math.radians(z_rotation_deg)
    Rz_tool = np.array([
        [math.cos(angle_rad), -math.sin(angle_rad), 0],
        [math.sin(angle_rad),  math.cos(angle_rad), 0],
        [0,                    0,                   1]
    ])
    
    # Apply rotation: R_new = R_current * Rz_tool
    # (rotate in tool frame means post-multiply)
    R_new = np.matmul(R_current, Rz_tool)
    
    # Convert back to rotation vector
    rx_new, ry_new, rz_new = matrix_to_rotvec(R_new)
    
    # Create target pose
    target_pose = [x, y, z, rx_new, ry_new, rz_new]
    
    print(f"Rotating tool {z_rotation_deg}° around Z-axis...")
    
    # Move to target pose using movel
    result = robot.movel(target_pose, a=a, v=v)
    
    if result == 0:
        print(f"✅ Tool orientation corrected successfully (rotated {z_rotation_deg}° around Z-axis)")
        return 0
    else:
        print(f"❌ Failed to correct tool orientation (result: {result})")
        return -1


def tcp_normalize(robot, x_axis=[0, -1, 0], y_axis=[-1, 0, 0], z_axis=[0, 0, -1], a=0.5, v=0.2):
    """
    Normalize TCP orientation according to specified axis alignment.
    """
    # Get current TCP pose
    current_pose = robot.get_actual_tcp_pose()
    if current_pose is None:
        print("[ERROR] Failed to get current TCP pose")
        return -1
    
    # Extract position (keep the same)
    x, y, z = current_pose[0], current_pose[1], current_pose[2]
    
    # Build rotation matrix from the desired axes
    # The input axes define how TCP axes should align with base axes
    x_vec = np.array(x_axis, dtype=float)
    y_vec = np.array(y_axis, dtype=float)
    z_vec = np.array(z_axis, dtype=float)
    
    # Normalize the vectors
    x_vec = x_vec / np.linalg.norm(x_vec)
    y_vec = y_vec / np.linalg.norm(y_vec)
    z_vec = z_vec / np.linalg.norm(z_vec)
    
    # Check orthogonality
    dot_xy = np.dot(x_vec, y_vec)
    dot_xz = np.dot(x_vec, z_vec)
    dot_yz = np.dot(y_vec, z_vec)
    
    if abs(dot_xy) > 0.01 or abs(dot_xz) > 0.01 or abs(dot_yz) > 0.01:
        print(f'[WARN] Input axes are not orthogonal! dot(x,y)={dot_xy:.4f}, dot(x,z)={dot_xz:.4f}, dot(y,z)={dot_yz:.4f}')
        # Orthogonalize using Gram-Schmidt process
        y_vec = y_vec - np.dot(y_vec, x_vec) * x_vec
        y_vec = y_vec / np.linalg.norm(y_vec)
        z_vec = np.cross(x_vec, y_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)
        print('[INFO] Axes orthogonalized using Gram-Schmidt process')
    
    # Construct rotation matrix (columns are the axis vectors)
    R_target = np.column_stack([x_vec, y_vec, z_vec])
    
    # Verify it's a valid rotation matrix (det should be 1)
    det = np.linalg.det(R_target)
    if abs(det - 1.0) > 0.01:
        print(f'[ERROR] Invalid rotation matrix! det={det:.4f}. Check if axes form a right-handed coordinate system.')
        return -1
    
    # Convert rotation matrix to axis-angle representation (rotation vector)
    rx, ry, rz = matrix_to_rotvec(R_target)
    
    # Create target pose
    target_pose = [x, y, z, rx, ry, rz]
    
    print(f"Normalizing TCP orientation (a={a}, v={v})...")
    
    # Move to target pose using movel (linear movement in base frame)
    result = robot.movel(target_pose, a=a, v=v)
    
    if result == 0:
        print("✅ TCP normalized successfully")
        return 0
    else:
        print(f"❌ Failed to normalize TCP (result: {result})")
        return -1

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

        # #  2.1 move to task pose
        # res = robot.movej([-0.6656, -1.772, -1.250, -1.522, 1.635, -0.6822], a=0.5, v=0.3)
        # print(f"MoveJ result: {res}")
        # time.sleep(1)

        # 3. enter freedrive mode f
        freedrive_duration = 10 #seconds
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

        # # # 5.1 Test movel function
        # actual_pose = robot.get_actual_tcp_pose()
        # actual_pose[0] -= 0.05  # Move 5 cm in X direction
        # res = robot.movel(actual_pose, a=0.2, v=0.1)

        # # 6. move tcp
        # res = robot.move_tcp([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], a=0.2, v=0.1)
        # print(f"Move TCP result: {res}")
        # time.sleep(1)

        # # 6.1 Test tcp_normalize function
        # res = tcp_normalize(robot, x_axis=[0, -1, 0], y_axis=[-1, 0, 0], z_axis=[0, 0, -1], a=0.5, v=0.2)
        # print(f"TCP normalize result: {res}")
        # time.sleep(2)
        # # 6.2 correct tool orientation (rotate 30 degrees around Z-axis)
        # res = tool_orientation_correction(robot, z_rotation_deg=30, a=0.5, v=0.2)
        # print(f"Tool orientation correction result: {res}")
        # time.sleep(2)

        # # 6.2 Test tool orientation correction (rotate 30 degrees around Z-axis)
        # res = tool_orientation_correction(robot, z_rotation_deg=30, a=0.2, v=0.2)
        # print(f"Tool orientation correction result: {res}")
        # time.sleep(2)
        # # Rotate back -30 degrees
        # res = tool_orientation_correction(robot, z_rotation_deg=-30, a=0.2, v=0.2)
        # print(f"Tool orientation correction result: {res}")

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
        
        # # 10. set active tool offset, it will not be affected by the configuration in installation-TCP
        # toolPullPush_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        # res = robot.set_tcp(toolPullPush_offset,tcp_name="toolPullPush")
        # time.sleep(1)

        # res = robot.move_tcp([0, 0, 0, 0.5236/2, 0, 0], a=0.2, v=0.2)
        # time.sleep(2)

        # res = robot.move_tcp([0, 0, 0, -0.5236/2, 0, 0], a=0.2, v=0.2)
        # time.sleep(1)
        # print(f"Move TCP with tool offset result: {res}")
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