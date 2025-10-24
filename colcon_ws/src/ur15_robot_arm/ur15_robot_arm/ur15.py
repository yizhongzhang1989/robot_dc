#!/usr/bin/env python3
"""
UR15 Robot Control Class
Establishes connection to UR robot and provides common URScript control functions.
"""

import socket
import time
import sys

class UR15Robot:
    def __init__(self, ip, port):
        """Initialize UR15 connection parameters"""
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False

    # ------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------
    def open(self):
        """Open TCP connection to robot"""
        try:
            print(f"[INFO] Connecting to UR15 robot at {self.ip}:{self.port} ...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)
            self.socket.connect((self.ip, self.port))
            self.connected = True
            print("[OK] Successfully connected to UR15 robot!")
            return 0
        except socket.timeout:
            print("[ERROR] Connection timeout - check network or robot IP.")
        except ConnectionRefusedError:
            print("[ERROR] Connection refused - check if robot is in Remote Control mode.")
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
        self._safe_close()
        return -1

    def close(self):
        """Close TCP connection"""
        try:
            if self.socket:
                # Brief delay before closing to ensure pending commands complete
                time.sleep(0.1)
                self.socket.close()
                print("[INFO] Disconnected from UR15 robot.")
        except Exception as e:
            print(f"[WARN] Error during disconnect: {e}")
        self._safe_close()
        return 0

    def _safe_close(self):
        """Internal: safely close socket"""
        self.connected = False
        self.socket = None

    def powerdown(self):
        """
        Shut down the robot, and power off the robot and controller.
        WARNING: This will completely power down the robot and controller. You will need to manually power it back on.
        """
        return self._send_command("powerdown()")

    # ------------------------------------------------------------
    # Internal communication helper
    # ------------------------------------------------------------
    def _send_command(self, command):
        """Send URScript command to robot"""
        if not self.connected:
            print("[WARN] Robot not connected.")
            return -1

        # Wrap command in a complete URScript program
        script = f"def myProg():\n  {command}\nend\n"
        
        try:
            self.socket.sendall(script.encode('utf-8'))
            # Brief delay to ensure command is processed
            time.sleep(0.05)
            return 0
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")
            return -1

    def _read_realtime_data(self, data_type):
        """Read data from the real-time interface (port 30003)"""
        if not self.connected:
            print("[WARN] Robot not connected.")
            return None
        
        try:
            # Connect to the real-time data interface
            rt_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            rt_socket.settimeout(2)
            rt_socket.connect((self.ip, 30003))
            
            # Read one packet of data
            data = rt_socket.recv(1116)  # Standard packet size for port 30003
            rt_socket.close()
            
            if len(data) < 1116:
                print(f"[WARN] Received incomplete data: {len(data)} bytes")
                return None
            
            import struct
            
            # Parse based on data type
            # Offsets based on UR real-time interface port 30003
            if data_type == "actual_joint_positions":
                # Bytes 252-299 contain actual joint positions (6 doubles)
                offset = 252
            elif data_type == "target_joint_positions":
                # Bytes 12-59 contain target joint positions (6 doubles)
                offset = 12
            elif data_type == "actual_tcp_pose":
                # Bytes 444-491 contain the actual TCP pose (6 doubles)
                offset = 444
            elif data_type == "target_tcp_pose":
                # Bytes 588-635 contain the target TCP pose (6 doubles)
                offset = 588
            elif data_type == "tcp_force":
                # Bytes 540-587 contain TCP force/torque (6 doubles)
                offset = 540
            else:
                print(f"[ERROR] Unknown data type: {data_type}")
                return None
            
            # Extract 6 double values
            result = []
            for i in range(6):
                value = struct.unpack('!d', data[offset + i*8:offset + (i+1)*8])[0]
                result.append(value)
            
            return result
            
        except socket.timeout:
            print("[ERROR] Timeout reading from real-time interface")
            return None
        except Exception as e:
            print(f"[ERROR] Query failed: {e}")
            return None

    def send_urscript(self, script):
        """Send a custom URScript string"""
        return self._send_command(script)

    # ------------------------------------------------------------
    # Basic message commands
    # ------------------------------------------------------------
    def popup(self, text, title="Popup", warning=False, error=False, blocking=False):
        """
        Display popup message on robot teach pendant
        blocking = true -> popup must be cleared before other actions will be performed.
        """
        cmd = f'popup("{text}"'
        if title:
            cmd += f', title="{title}"'
        cmd += f', warning={warning}, error={error}, blocking={blocking})'
        return self._send_command(cmd)

    def textmsg(self, *args):
        """Output text message to log"""
        args_str = ", ".join([f'"{a}"' for a in args])
        return self._send_command(f"textmsg({args_str})")

    # ------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------
    def movej(self, q, a=1.4, v=1.05, t=0, r=0):
        """
        Move to position (linear in joint-space)
        
        Note that the zero position of UR15 is [0, -1.57, 0, -1.57, 0, 0]
        """
        # Format joint positions as URScript list
        if isinstance(q, (list, tuple)):
            q_str = "[" + ",".join([str(val) for val in q]) + "]"
        else:
            q_str = str(q)
        
        # Build URScript command
        cmd = f"movej({q_str}, a={a}, v={v}, t={t}, r={r})"
        return self._send_command(cmd)

    def movel(self, pose, a=1.2, v=0.25, t=0, r=0):
        """
        Move to position (linear in tool-space)
        """
        # Format pose
        if isinstance(pose, (list, tuple)):
            pose_str = "p[" + ",".join([str(val) for val in pose]) + "]"
        elif isinstance(pose, str):
            pose_str = pose
        else:
            pose_str = str(pose)
        
        # Build URScript command
        cmd = f"movel({pose_str}, a={a}, v={v}, t={t}, r={r})"
        return self._send_command(cmd)

    def move_tcp(self, offset, a=1.2, v=0.25, t=0, r=0):
        """
        Move TCP along its own coordinate frame (relative movement)
        """
        # Get current TCP pose
        current_pose = self.get_actual_tcp_pose()
        if current_pose is None:
            print("[ERROR] Failed to get current TCP pose")
            return -1
        
        # Calculate target pose using class method
        # pose_trans transforms offset from TCP frame to base frame
        target_pose = self.pose_trans(current_pose, offset)
        
        # Move to target pose
        return self.movel(target_pose, a=a, v=v, t=t, r=r)

    def servoj(self, q, a=0, v=0, t=0.002, lookahead_time=0.1, gain=300):
        """
        Servoj - Online realtime control of joint positions
        """
        # Format joint positions as URScript list
        if isinstance(q, (list, tuple)):
            q_str = "[" + ",".join([str(val) for val in q]) + "]"
        else:
            q_str = str(q)
        
        # Build URScript command
        cmd = f"servoj({q_str}, {a}, {v}, {t}, {lookahead_time}, {gain})"
        return self._send_command(cmd)

    def freedrive_mode(self, freeAxes=None, feature=None, duration=0):
        """
        Set robot in freedrive mode     
        Returns:
            0 on success, -1 on failure
        """
        # Set defaults
        if freeAxes is None:
            freeAxes = [1, 1, 1, 1, 1, 1]
        if feature is None:
            feature = "p[0,0,0,0,0,0]"
        
        # Format freeAxes as URScript list
        if isinstance(freeAxes, (list, tuple)):
            axes_str = "[" + ",".join([str(val) for val in freeAxes]) + "]"
        else:
            axes_str = str(freeAxes)
        
        # Format feature
        if isinstance(feature, str):
            if feature in ["base", "tool"]:
                feature_str = f'"{feature}"'
            else:
                # Already a string like "p[0,0,0,0,0,0]"
                feature_str = feature
        elif isinstance(feature, (list, tuple)):
            feature_str = "p[" + ",".join([str(val) for val in feature]) + "]"
        else:
            feature_str = str(feature)
        
        # Build URScript command with a loop to keep it running
        if duration > 0:
            # Run for specific duration
            cmd = f"freedrive_mode({axes_str}, {feature_str})\n  sleep({duration})\n  end_freedrive_mode()"
            result = self._send_command(cmd)
            # Wait in Python for the freedrive duration
            if result == 0:
                time.sleep(duration)
            return result
        else:
            # Run indefinitely until end_freedrive_mode is called
            cmd = f"freedrive_mode({axes_str}, {feature_str})\n  while True:\n    sleep(0.1)\n  end"
            return self._send_command(cmd)

    def end_freedrive_mode(self):
        """
        Stop freedrive mode
        """
        return self._send_command("end_freedrive_mode()")

    # ------------------------------------------------------------
    # Tool control commands
    # ------------------------------------------------------------
    def set_tool_voltage(self, voltage):
        """
        Sets the voltage level for the power supply that delivers power to the connector plug 
        in the tool flange of the robot. The voltage can be 0, 12 or 24 volts.
        """
        # Validate voltage parameter
        if voltage not in [0, 12, 24]:
            print(f"[ERROR] Invalid voltage value: {voltage}. Must be 0, 12, or 24.")
            return -1
        
        # Build URScript command with sleep to allow the voltage to be applied
        # The sleep ensures the robot has time to actually set the voltage before the program ends
        cmd = f"set_tool_voltage({voltage})\n  sleep(1)"
        return self._send_command(cmd)

    def set_tcp(self, pose, tcp_name=""):
        """
        Sets the active TCP offset, i.e., the transformation from the output flange 
        """
        # Format pose
        if isinstance(pose, (list, tuple)):
            if len(pose) != 6:
                print(f"[ERROR] Invalid pose: {pose}. Must have 6 elements [x,y,z,rx,ry,rz].")
                return -1
            pose_str = "p[" + ",".join([str(val) for val in pose]) + "]"
        elif isinstance(pose, str):
            pose_str = pose
        else:
            print(f"[ERROR] Invalid pose type: {type(pose)}. Must be list, tuple, or string.")
            return -1
        
        # Validate tcp_name parameter
        if not isinstance(tcp_name, str):
            print(f"[ERROR] Invalid tcp_name: {tcp_name}. Must be a string.")
            return -1
        
        # Build URScript command
        if tcp_name:
            cmd = f'set_tcp({pose_str}, "{tcp_name}")'
        else:
            cmd = f'set_tcp({pose_str})'
        
        result = self._send_command(cmd)
        
        if result == 0:
            if tcp_name:
                print(f"[INFO] TCP set to {pose_str} with name '{tcp_name}'")
            else:
                print(f"[INFO] TCP set to {pose_str}")
        
        return result

    def socket_open(self, address, port, socket_name='socket_0'):
        """
        Open TCP/IP ethernet communication socket.
        """
        # Validate parameters
        if not isinstance(address, str) or not address:
            print(f"[ERROR] Invalid address: {address}. Must be a non-empty string.")
            return False
        
        if not isinstance(port, int) or port < 1 or port > 65535:
            print(f"[ERROR] Invalid port: {port}. Must be an integer between 1 and 65535.")
            return False
        
        if not isinstance(socket_name, str) or not socket_name:
            print(f"[ERROR] Invalid socket_name: {socket_name}. Must be a non-empty string.")
            return False
        
        # Build URScript command
        # socket_open returns True/False, we need to capture and return this
        cmd = f'socket_open("{address}", {port}, "{socket_name}")'
        
        result = self._send_command(cmd)
    
        if result == 0:
            print(f"[INFO] Socket '{socket_name}' opened to {address}:{port}")
            return True
        else:
            print(f"[ERROR] Failed to send socket_open command")
            return False

    def socket_send_byte(self, value, socket_name='socket_0'):
        """
        Sends a byte to the server.
        """
        # Validate value parameter
        if not isinstance(value, int) or value < 0 or value > 255:
            print(f"[ERROR] Invalid value: {value}. Must be an integer between 0 and 255 (byte range).")
            return False
        
        # Validate socket_name parameter
        if not isinstance(socket_name, str) or not socket_name:
            print(f"[ERROR] Invalid socket_name: {socket_name}. Must be a non-empty string.")
            return False
        
        # Build URScript command
        cmd = f'socket_send_byte({value}, "{socket_name}")'
        
        result = self._send_command(cmd)
        
        if result == 0:
            return True
        else:
            print(f"[ERROR] Failed to send byte {value} through socket '{socket_name}'")
            return False

    def socket_close(self, socket_name='socket_0'):
        """
        Closes TCP/IP socket communication.
        """
        # Validate socket_name parameter
        if not isinstance(socket_name, str) or not socket_name:
            print(f"[ERROR] Invalid socket_name: {socket_name}. Must be a non-empty string.")
            return -1
        
        # Build URScript command
        cmd = f'socket_close("{socket_name}")'
        
        result = self._send_command(cmd)
        
        if result == 0:
            print(f"[INFO] Socket '{socket_name}' closed")
        
        return result

    def socket_send_all(self, address, port, data, socket_name='socket_0'):
        """
        Open socket, send data, and close socket in one atomic operation.
        """
        # Validate parameters (reuse validation from other methods)
        if not isinstance(address, str) or not address:
            print(f"[ERROR] Invalid address: {address}. Must be a non-empty string.")
            return False
        
        if not isinstance(port, int) or port < 1 or port > 65535:
            print(f"[ERROR] Invalid port: {port}. Must be an integer between 1 and 65535.")
            return False
        
        if not isinstance(data, (list, tuple)) or len(data) == 0:
            print(f"[ERROR] Invalid data. Must be a non-empty list or tuple of byte values.")
            return False
        
        for i, value in enumerate(data):
            if not isinstance(value, int) or value < 0 or value > 255:
                print(f"[ERROR] Invalid byte at index {i}: {value}. Must be 0-255.")
                return False
        
        if not isinstance(socket_name, str) or not socket_name:
            print(f"[ERROR] Invalid socket_name: {socket_name}. Must be a non-empty string.")
            return False
        
        # Build complete URScript program
        commands = []
        commands.append(f'socket_open("{address}", {port}, "{socket_name}")')
        commands.append('sleep(0.5)')  # Wait for connection to establish
        
        for value in data:
            commands.append(f'socket_send_byte({value}, "{socket_name}")')
        
        commands.append('sync()')  # Ensure all data is sent
        commands.append('sleep(0.5)')  # Wait for data to be transmitted
        commands.append(f'socket_close("{socket_name}")')
        
        full_cmd = "\n  ".join(commands)
        
        result = self._send_command(full_cmd)
        
        if result == 0:
            print(f"[INFO] Sent {len(data)} bytes to {address}:{port}")
            return True
        else:
            print(f"[ERROR] Failed to send data")
            return False

    # ------------------------------------------------------------
    # Query commands
    # ------------------------------------------------------------
    def get_target_tcp_pose(self):
        """
        Get the current target tool pose
        """
        return self._read_realtime_data("target_tcp_pose")

    def get_target_joint_positions(self):
        """
        Get the desired angular position of all joints
        """
        return self._read_realtime_data("target_joint_positions")

    def get_actual_tcp_pose(self):
        """
        Get the current measured tool pose
        """
        return self._read_realtime_data("actual_tcp_pose")

    def get_actual_joint_positions(self):
        """
        Get the actual angular positions of all joints
        """
        return self._read_realtime_data("actual_joint_positions")

    def get_tcp_force(self):
        """
        Get the force/torque vector at the tool flange
        """
        return self._read_realtime_data("tcp_force")

    # ------------------------------------------------------------
    # Transformation functions
    # ------------------------------------------------------------
    def pose_trans(self, p_from, p_from_to):
        """
        Pose transformation: transform p_from_to (in frame of p_from) to base frame
        """
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
            angle = math.acos((np.trace(R) - 1) / 2)
            
            if angle < 1e-10:
                return 0, 0, 0
            
            if abs(angle - math.pi) < 1e-10:
                # Special case: 180 degree rotation
                # Find the axis from the diagonal elements
                if R[0,0] >= R[1,1] and R[0,0] >= R[2,2]:
                    kx = math.sqrt((R[0,0] + 1) / 2)
                    ky = R[0,1] / (2 * kx)
                    kz = R[0,2] / (2 * kx)
                elif R[1,1] >= R[2,2]:
                    ky = math.sqrt((R[1,1] + 1) / 2)
                    kx = R[0,1] / (2 * ky)
                    kz = R[1,2] / (2 * ky)
                else:
                    kz = math.sqrt((R[2,2] + 1) / 2)
                    kx = R[0,2] / (2 * kz)
                    ky = R[1,2] / (2 * kz)
            else:
                # General case
                denom = 2 * math.sin(angle)
                kx = (R[2,1] - R[1,2]) / denom
                ky = (R[0,2] - R[2,0]) / denom
                kz = (R[1,0] - R[0,1]) / denom
            
            return kx * angle, ky * angle, kz * angle
        
        def pose_to_matrix(pose):
            """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
            T = np.eye(4)
            T[0:3, 0:3] = rotvec_to_matrix(pose[3], pose[4], pose[5])
            T[0:3, 3] = [pose[0], pose[1], pose[2]]
            return T
        
        def matrix_to_pose(T):
            """Convert 4x4 homogeneous transformation matrix to pose [X,Y,Z,Rx,Ry,Rz]"""
            x, y, z = T[0:3, 3]
            rx, ry, rz = matrix_to_rotvec(T[0:3, 0:3])
            return [x, y, z, rx, ry, rz]
        
        # Convert poses to transformation matrices
        T_from = pose_to_matrix(p_from)
        T_from_to = pose_to_matrix(p_from_to)
        
        # Multiply matrices: T_result = T_from * T_from_to
        T_result = np.matmul(T_from, T_from_to)
        
        # Convert back to pose
        result = matrix_to_pose(T_result)
        
        return result

