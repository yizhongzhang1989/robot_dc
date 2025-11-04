#!/usr/bin/env python3
"""
UR15 Robot Control Class
Establishes connection to UR robot and provides common URScript control functions.
"""

import socket
import time
import sys
import math
import numpy as np


# ------------------------------------------------------------
# Helper functions for rotation calculations
# ------------------------------------------------------------
def rotvec_to_matrix(rx, ry, rz):
    """Convert rotation vector to rotation matrix using Rodrigues' formula"""
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
    angle = math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    
    if angle < 1e-10:
        return np.array([0, 0, 0])
    
    if abs(angle - math.pi) < 1e-10:
        # Special case: 180 degree rotation
        # Find the axis from the diagonal elements
        if R[0,0] >= R[1,1] and R[0,0] >= R[2,2]:
            kx = math.sqrt((R[0,0] + 1) / 2)
            ky = R[0,1] / (2 * kx) if kx > 1e-10 else 0
            kz = R[0,2] / (2 * kx) if kx > 1e-10 else 0
        elif R[1,1] >= R[2,2]:
            ky = math.sqrt((R[1,1] + 1) / 2)
            kx = R[0,1] / (2 * ky) if ky > 1e-10 else 0
            kz = R[1,2] / (2 * ky) if ky > 1e-10 else 0
        else:
            kz = math.sqrt((R[2,2] + 1) / 2)
            kx = R[0,2] / (2 * kz) if kz > 1e-10 else 0
            ky = R[1,2] / (2 * kz) if kz > 1e-10 else 0
        return np.array([kx * angle, ky * angle, kz * angle])
    else:
        # General case
        denom = 2 * math.sin(angle)
        kx = (R[2,1] - R[1,2]) / denom
        ky = (R[0,2] - R[2,0]) / denom
        kz = (R[1,0] - R[0,1]) / denom
        return np.array([kx * angle, ky * angle, kz * angle])


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
    def movej(self, q, a=1.4, v=1.05, t=0, r=0, blocking=True, threshold=0.01):
        """
        Move to position (linear in joint-space)
        
        Note that the zero position of UR15 is [0, -1.57, 0, -1.57, 0, 0]
        
        Args:
            q: Target joint positions [j0, j1, j2, j3, j4, j5] in radians
            a: Joint acceleration [rad/s^2]
            v: Joint velocity [rad/s]
            t: Time [s]
            r: Blend radius [m]
            blocking: If True, wait until motion completes (default: True)
            threshold: Joint position threshold in radians to consider motion complete (default: 0.01)
        """
        # Format joint positions as URScript list
        if isinstance(q, (list, tuple)):
            q_str = "[" + ",".join([str(val) for val in q]) + "]"
            target_q = list(q)
        else:
            q_str = str(q)
            target_q = None
        
        # Build URScript command
        cmd = f"movej({q_str}, a={a}, v={v}, t={t}, r={r})"
        result = self._send_command(cmd)
        
        if result != 0:
            return result
        
        # If blocking mode and we have target positions, wait for completion
        if blocking and target_q is not None:
            import math
            time.sleep(0.1)  # Initial delay to let motion start
            
            max_wait_time = 60  # Maximum wait time in seconds
            start_time = time.time()
            
            while time.time() - start_time < max_wait_time:
                actual_q = self.get_actual_joint_positions()
                if actual_q is None:
                    print("[WARN] Failed to read joint positions during blocking wait")
                    time.sleep(0.1)
                    continue
                
                # Check if all joints are within threshold
                all_close = True
                for i in range(6):
                    if abs(actual_q[i] - target_q[i]) > threshold:
                        all_close = False
                        break
                
                if all_close:
                    # Motion complete
                    return 0
                
                time.sleep(0.05)  # Check every 50ms
            
            print(f"[WARN] movej blocking timeout after {max_wait_time}s")
            return -1
        
        return result

    def movel(self, pose, a=1.2, v=0.25, t=0, r=0, blocking=True, threshold=0.001):
        """
        Move to position (linear in tool-space)
        
        Args:
            pose: Target TCP pose [x,y,z,rx,ry,rz] in meters and radians
            a: Tool acceleration [m/s^2]
            v: Tool velocity [m/s]
            t: Time [s]
            r: Blend radius [m]
            blocking: If True, wait until motion completes (default: True)
            threshold: Position threshold in meters for xyz and radians for rotation (default: 0.001)
        """
        # Format pose
        target_pose = None
        if isinstance(pose, (list, tuple)):
            pose_str = "p[" + ",".join([str(val) for val in pose]) + "]"
            target_pose = list(pose)
        elif isinstance(pose, str):
            pose_str = pose
        else:
            pose_str = str(pose)
        
        # Build URScript command
        cmd = f"movel({pose_str}, a={a}, v={v}, t={t}, r={r})"
        result = self._send_command(cmd)
        
        if result != 0:
            return result
        
        # If blocking mode and we have target pose, wait for completion
        if blocking and target_pose is not None:
            import math
            time.sleep(0.1)  # Initial delay to let motion start
            
            max_wait_time = 60  # Maximum wait time in seconds
            start_time = time.time()
            
            while time.time() - start_time < max_wait_time:
                actual_pose = self.get_actual_tcp_pose()
                if actual_pose is None:
                    print("[WARN] Failed to read TCP pose during blocking wait")
                    time.sleep(0.1)
                    continue
                
                # Check if all pose elements are within threshold
                all_close = True
                for i in range(6):
                    if abs(actual_pose[i] - target_pose[i]) > threshold:
                        all_close = False
                        break
                
                if all_close:
                    # Motion complete
                    return 0
                
                time.sleep(0.05)  # Check every 50ms
            
            print(f"[WARN] movel blocking timeout after {max_wait_time}s")
            return -1
        
        return result

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

    def force_mode(self, task_frame, selection_vector, wrench, type=2, limits=None):
        """
        Set robot to be controlled in force mode
        """
        # Set default limits if not provided
        if limits is None:
            limits = [2, 2, 1.5, 1, 1, 1]
        
        # Validate type parameter
        if not isinstance(type, int) or type < 1 or type > 3:
            print(f"[ERROR] Invalid type: {type}. Must be an integer between 1 and 3.")
            return -1
        
        # Format task_frame
        if isinstance(task_frame, (list, tuple)):
            if len(task_frame) != 6:
                print(f"[ERROR] Invalid task_frame: {task_frame}. Must have 6 elements [x,y,z,rx,ry,rz].")
                return -1
            task_frame_str = "p[" + ",".join([str(val) for val in task_frame]) + "]"
        elif isinstance(task_frame, str):
            task_frame_str = task_frame
        else:
            print(f"[ERROR] Invalid task_frame type: {type(task_frame)}. Must be list, tuple, or string.")
            return -1
        
        # Format selection_vector
        if isinstance(selection_vector, (list, tuple)):
            if len(selection_vector) != 6:
                print(f"[ERROR] Invalid selection_vector: {selection_vector}. Must have 6 elements.")
                return -1
            # Validate values are 0 or 1
            for i, val in enumerate(selection_vector):
                if val not in [0, 1]:
                    print(f"[ERROR] Invalid selection_vector value at index {i}: {val}. Must be 0 or 1.")
                    return -1
            selection_vector_str = "[" + ",".join([str(val) for val in selection_vector]) + "]"
        else:
            print(f"[ERROR] Invalid selection_vector type: {type(selection_vector)}. Must be list or tuple.")
            return -1
        
        # Format wrench
        if isinstance(wrench, (list, tuple)):
            if len(wrench) != 6:
                print(f"[ERROR] Invalid wrench: {wrench}. Must have 6 elements [fx,fy,fz,tx,ty,tz].")
                return -1
            wrench_str = "[" + ",".join([str(val) for val in wrench]) + "]"
        else:
            print(f"[ERROR] Invalid wrench type: {type(wrench)}. Must be list or tuple.")
            return -1
        
        # Format limits
        if isinstance(limits, (list, tuple)):
            if len(limits) != 6:
                print(f"[ERROR] Invalid limits: {limits}. Must have 6 elements.")
                return -1
            limits_str = "[" + ",".join([str(val) for val in limits]) + "]"
        else:
            print(f"[ERROR] Invalid limits type: {type(limits)}. Must be list or tuple.")
            return -1
        
        # Build complete URScript program to keep force_mode running
        # Force mode needs to be called continuously to remain active
        script = "def myProg():\n"
        script += "  while (True):\n"
        script += f"    force_mode({task_frame_str}, {selection_vector_str}, {wrench_str}, {type}, {limits_str})\n"
        script += "    sync()\n"
        script += "  end\n"
        script += "end\n"
        
        # Send directly without using _send_command to avoid double wrapping
        if not self.connected:
            print("[WARN] Robot not connected.")
            return -1
        
        try:
            self.socket.sendall(script.encode('utf-8'))
            time.sleep(0.1)
            print(f"[INFO] Force mode activated with type={type}")
            return 0
        except Exception as e:
            print(f"[ERROR] Failed to send force_mode command: {e}")
            return -1

    def end_force_mode(self):
        """
        Stop force mode and return to normal operation
        
        Returns:
            0 on success, -1 on failure
        """
        result = self._send_command("end_force_mode()")
        
        if result == 0:
            print("\n[INFO] Force mode deactivated")
        
        return result

    def force_mode_set_gain_scaling(self, scaling=1.0):
        """
        Scales the gain in force mode.
        """
        # Validate scaling parameter
        if not isinstance(scaling, (int, float)):
            print(f"[ERROR] Invalid scaling type: {type(scaling)}. Must be a number.")
            return -1
        
        if scaling < 0 or scaling > 2:
            print(f"[ERROR] Invalid scaling value: {scaling}. Must be between 0 and 2.")
            return -1
        
        # Build URScript command
        cmd = f"force_mode_set_gain_scaling({scaling})"
        
        result = self._send_command(cmd)
        
        if result == 0:
            print(f"[INFO] Force mode gain scaling set to {scaling}")
        
        return result

    def force_mode_set_damping(self, damping=0.005):
        """
        Sets the damping parameter in force mode.
        """
        # Validate damping parameter
        if not isinstance(damping, (int, float)):
            print(f"[ERROR] Invalid damping type: {type(damping)}. Must be a number.")
            return -1
        
        if damping < 0 or damping > 1:
            print(f"[ERROR] Invalid damping value: {damping}. Must be between 0 and 1.")
            return -1
        
        # Build URScript command
        cmd = f"force_mode_set_damping({damping})"
        
        result = self._send_command(cmd)
        
        if result == 0:
            print(f"[INFO] Force mode damping set to {damping}")
        
        return result

    def force_control_task(self, task_frame, selection_vector, wrench, type=2, limits=None, 
                            scaling=1.0, damping=0.005, end_type=0, end_time=None, end_force=None, end_distance=None, end_angle=None):
        """
        High-level force control task that combines force mode configuration and execution.
        
        Args:
            task_frame: A pose vector that defines the force frame relative to the base frame
            selection_vector: A 6d vector of 0s and 1s. 1 means that the robot will be 
                            compliant in the corresponding axis of the task frame
            wrench: The forces/torques the robot will apply to its environment [fx,fy,fz,tx,ty,tz]
            type: An integer [1;3] specifying how the robot interprets the force frame
            limits: A 6d vector with float values that are interpreted differently for 
                   compliant/non-compliant axes
            scaling: Gain scaling factor for force mode (0.0 to 2.0)
            damping: Damping parameter for force mode (0.0 to 1.0)
            end_type: Termination condition type
                     0 = manual termination (user must call end_force_mode() externally)
                     1 = time-based termination using end_time parameter (end_time is required)
                     2 = force/torque-based termination using end_force parameter (end_force is required)
                     3 = displacement-based termination using end_distance parameter (end_distance is required)
                     4 = rotation angle-based termination using end_angle parameter (end_angle is required)
            end_time: Time duration in seconds for force control (required when end_type=1, ignored otherwise)
            end_force: Maximum force/torque thresholds [fx,fy,fz,tx,ty,tz] for termination 
                      (required when end_type=2, only checked for axes where selection_vector=1)
            end_distance: Maximum TCP displacement [dx,dy,dz,drx,dry,drz] for termination in meters/radians
                         (required when end_type=3, only xyz displacement is checked for axes where selection_vector=1)
            end_angle: Maximum rotation angle around TCP z-axis for termination in degrees
                      (required when end_type=4)
        
        Returns:
            0 on success, -1 on failure
        """
        # Validate end_type
        if end_type not in [0, 1, 2, 3, 4]:
            print(f"[ERROR] Invalid end_type: {end_type}. Must be 0 (manual), 1 (time-based), 2 (force-based), 3 (displacement-based), or 4 (rotation-based).")
            return -1
        
        # Validate end_time for time-based termination
        if end_type == 1:
            if end_time is None:
                print(f"[ERROR] end_time is required when end_type=1 (time-based termination).")
                return -1
            if not isinstance(end_time, (int, float)) or end_time <= 0:
                print(f"[ERROR] Invalid end_time: {end_time}. Must be a positive number.")
                return -1
        
        # Validate end_force for force-based termination
        if end_type == 2:
            if end_force is None:
                print(f"[ERROR] end_force is required when end_type=2 (force-based termination).")
                return -1
            if not isinstance(end_force, (list, tuple)) or len(end_force) != 6:
                print(f"[ERROR] Invalid end_force: {end_force}. Must be a 6-element list/tuple [fx,fy,fz,tx,ty,tz].")
                return -1
            # Validate that end_force values are numbers
            for i, val in enumerate(end_force):
                if not isinstance(val, (int, float)):
                    print(f"[ERROR] Invalid end_force value at index {i}: {val}. Must be a number.")
                    return -1
        
        # Validate end_distance for displacement-based termination
        if end_type == 3:
            if end_distance is None:
                print(f"[ERROR] end_distance is required when end_type=3 (displacement-based termination).")
                return -1
            if not isinstance(end_distance, (list, tuple)) or len(end_distance) != 6:
                print(f"[ERROR] Invalid end_distance: {end_distance}. Must be a 6-element list/tuple [dx,dy,dz,drx,dry,drz].")
                return -1
            # Validate that end_distance values are numbers
            for i, val in enumerate(end_distance):
                if not isinstance(val, (int, float)):
                    print(f"[ERROR] Invalid end_distance value at index {i}: {val}. Must be a number.")
                    return -1
        
        # Validate end_angle for rotation-based termination
        if end_type == 4:
            if end_angle is None:
                print(f"[ERROR] end_angle is required when end_type=4 (rotation-based termination).")
                return -1
            if not isinstance(end_angle, (int, float)):
                print(f"[ERROR] Invalid end_angle: {end_angle}. Must be a number (in degrees).")
                return -1
        
        # Step 0: Zero the force/torque sensor
        print("[INFO] Zeroing force/torque sensor...")
        result = self.zero_ftsensor()
        if result != 0:
            print("[ERROR] Failed to zero force/torque sensor")
            return -1
        time.sleep(0.5)  # Wait for sensor to stabilize
        
        # Step 1: Set gain scaling
        print(f"[INFO] Setting force mode gain scaling to {scaling}...")
        result = self.force_mode_set_gain_scaling(scaling)
        if result != 0:
            print("[ERROR] Failed to set gain scaling")
            return -1
        
        # Step 2: Set damping
        print(f"[INFO] Setting force mode damping to {damping}...")
        result = self.force_mode_set_damping(damping)
        if result != 0:
            print("[ERROR] Failed to set damping")
            return -1
        
        # Step 3: Activate force mode
        if end_type == 0:
            print("[INFO] Activating force mode (manual termination mode)...")
            print("[INFO] Call end_force_mode() to stop force control")
        elif end_type == 1:
            print(f"[INFO] Activating force mode for {end_time} seconds...")
        elif end_type == 2:
            print("[INFO] Activating force mode (force-based termination mode)...")
            print(f"[INFO] Will terminate when force thresholds are reached: {end_force}")
        elif end_type == 3:
            print("[INFO] Activating force mode (displacement-based termination mode)...")
            print(f"[INFO] Will terminate when displacement thresholds are reached: {end_distance}")
        elif end_type == 4:
            print("[INFO] Activating force mode (rotation angle-based termination mode)...")
            print(f"[INFO] Will terminate when rotation angle around TCP z-axis reaches: {end_angle} deg")
        
        result = self.force_mode(task_frame, selection_vector, wrench, type, limits)
        if result != 0:
            print("[ERROR] Failed to activate force mode")
            return -1
        
        # Step 4: Handle termination based on end_type
        if end_type == 0:
            # Manual termination - force mode remains active
            print("[INFO] Force mode activated. Waiting for external end_force_mode() call...")
            return 0
        
        elif end_type == 1:
            # Time-based termination
            print(f"[INFO] Force control running for {end_time} seconds...")
            time.sleep(end_time)
            
            # End force mode
            print("[INFO] Time elapsed, stopping force mode...")
            result = self.end_force_mode()
            if result != 0:
                print("[ERROR] Failed to end force mode")
                return -1
        
        elif end_type == 2:
            # Force-based termination
            print("[INFO] Monitoring force/torque for threshold detection...")
            print("[INFO] Call end_force_mode() externally to stop before threshold is reached")
            check_interval = 0.05  # Check every 50ms
            
            while True:
                # Read current TCP force
                current_force = self.get_tcp_force()
                if current_force is None:
                    print("[WARN] Failed to read TCP force during monitoring")
                    time.sleep(check_interval)
                    continue
                
                # Check each axis that has selection_vector=1
                threshold_reached = False
                for i in range(6):
                    if selection_vector[i] == 1:
                        # Check if force/torque exceeds threshold (absolute value)
                        if abs(current_force[i]) >= abs(end_force[i]):
                            axis_names = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
                            print(f"\n[INFO] Force threshold reached on {axis_names[i]}: "
                                  f"{current_force[i]:.2f} >= {end_force[i]:.2f}")
                            threshold_reached = True
                            break
                
                if threshold_reached:
                    # End force mode
                    print("[INFO] Force threshold reached, stopping force mode...")
                    result = self.end_force_mode()
                    if result != 0:
                        print("[ERROR] Failed to end force mode")
                        return -1
                    break
                
                time.sleep(check_interval)
        
        elif end_type == 3:
            # Displacement-based termination
            print("[INFO] Monitoring TCP displacement for threshold detection...")
            print("[INFO] Call end_force_mode() externally to stop before threshold is reached")
            
            # Record initial TCP pose
            initial_pose = self.get_actual_tcp_pose()
            if initial_pose is None:
                print("[ERROR] Failed to read initial TCP pose")
                self.end_force_mode()
                return -1
            
            # Extract task_frame rotation to transform displacement to task_frame coordinate system
            # Parse task_frame if it's a string
            if isinstance(task_frame, str):
                # Parse string like "p[x,y,z,rx,ry,rz]"
                import re
                match = re.search(r'p\[(.*?)\]', task_frame)
                if match:
                    task_frame_list = [float(x.strip()) for x in match.group(1).split(',')]
                else:
                    task_frame_list = [0, 0, 0, 0, 0, 0]
            else:
                task_frame_list = list(task_frame)
            
            # Build rotation matrix from task_frame orientation
            R_task = rotvec_to_matrix(task_frame_list[3], task_frame_list[4], task_frame_list[5])
            
            check_interval = 0.05  # Check every 50ms
            
            while True:
                # Read current TCP pose
                current_pose = self.get_actual_tcp_pose()
                if current_pose is None:
                    print("[WARN] Failed to read TCP pose during monitoring")
                    time.sleep(check_interval)
                    continue
                
                # Calculate displacement from initial pose in base frame
                displacement_base = np.array([current_pose[i] - initial_pose[i] for i in range(3)])
                
                # Transform displacement to task_frame coordinate system
                # displacement_task = R_task^T * displacement_base
                displacement_task = R_task.T @ displacement_base
                
                # Check each xyz axis that has selection_vector=1
                threshold_reached = False
                for i in range(3):
                    if selection_vector[i] == 1:
                        # Check if displacement exceeds threshold (absolute value)
                        if abs(displacement_task[i]) >= abs(end_distance[i]):
                            axis_names = ['X', 'Y', 'Z']
                            print(f"\n[INFO] Displacement threshold reached on task_frame {axis_names[i]}: "
                                  f"{displacement_task[i]:.4f}m >= {end_distance[i]:.4f}m")
                            threshold_reached = True
                            break
                
                if threshold_reached:
                    # End force mode
                    print("[INFO] Displacement threshold reached, stopping force mode...")
                    result = self.end_force_mode()
                    if result != 0:
                        print("[ERROR] Failed to end force mode")
                        return -1
                    break
                
                time.sleep(check_interval)
        
        elif end_type == 4:
            # Rotation angle-based termination
            print("[INFO] Monitoring TCP rotation angle around task_frame z-axis for threshold detection...")
            print("[INFO] Call end_force_mode() externally to stop before threshold is reached")
            
            # Record initial TCP pose
            initial_pose = self.get_actual_tcp_pose()
            if initial_pose is None:
                print("[ERROR] Failed to read initial TCP pose")
                self.end_force_mode()
                return -1
            
            # Extract task_frame rotation
            # Parse task_frame if it's a string
            if isinstance(task_frame, str):
                # Parse string like "p[x,y,z,rx,ry,rz]"
                import re
                match = re.search(r'p\[(.*?)\]', task_frame)
                if match:
                    task_frame_list = [float(x.strip()) for x in match.group(1).split(',')]
                else:
                    task_frame_list = [0, 0, 0, 0, 0, 0]
            else:
                task_frame_list = list(task_frame)
            
            # Build rotation matrix from task_frame orientation
            R_task = rotvec_to_matrix(task_frame_list[3], task_frame_list[4], task_frame_list[5])
            
            check_interval = 0.05  # Check every 50ms
            
            import math
            
            while True:
                # Read current TCP pose
                current_pose = self.get_actual_tcp_pose()
                if current_pose is None:
                    print("[WARN] Failed to read TCP pose during monitoring")
                    time.sleep(check_interval)
                    continue
                
                # Convert initial and current rotation vectors to rotation matrices (in base frame)
                R_initial = rotvec_to_matrix(initial_pose[3], initial_pose[4], initial_pose[5])
                R_current = rotvec_to_matrix(current_pose[3], current_pose[4], current_pose[5])
                
                # Calculate relative rotation in base frame: R_relative_base = R_initial^T * R_current
                R_relative_base = R_initial.T @ R_current
                
                # Transform relative rotation to task_frame coordinate system
                # R_relative_task = R_task^T * R_relative_base * R_task
                R_relative_task = R_task.T @ R_relative_base @ R_task
                
                # Convert relative rotation matrix back to rotation vector (in task_frame)
                rotvec_relative_task = matrix_to_rotvec(R_relative_task)
                
                # The z-component of this rotation vector is the rotation around task_frame z-axis
                rotation_around_task_z = rotvec_relative_task[2]
                
                rotation_angle_rad = abs(rotation_around_task_z)
                rotation_angle_deg = math.degrees(rotation_angle_rad)
                
                # Convert end_angle from degrees to radians for comparison
                end_angle_rad = math.radians(abs(end_angle))
                
                # Check if rotation angle exceeds threshold
                if rotation_angle_rad >= end_angle_rad:
                    print(f"\n[INFO] Rotation angle threshold reached around task_frame z-axis: "
                          f"{rotation_angle_deg:.2f} deg >= {abs(end_angle):.2f} deg")
                    
                    # End force mode
                    print("[INFO] Rotation threshold reached, stopping force mode...")
                    result = self.end_force_mode()
                    if result != 0:
                        print("[ERROR] Failed to end force mode")
                        return -1
                    break
                
                time.sleep(check_interval)
        
        print("[INFO] Force control task completed successfully")
        return 0

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

    def zero_ftsensor(self):
        """
        Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting 
        the current measurement from the subsequent measurements.
        
        Returns:
            0 on success, -1 on failure
        """
        return self._send_command("zero_ftsensor()")

    def high_holding_torque_disable(self):
        """
        Disable high holding torque mode. This allows the robot to be more compliant and reduces power consumption when holding position.
        """
        result = self._send_command("high_holding_torque_disable()")
        
        if result == 0:
            print("[INFO] High holding torque disabled")
        
        return result

    def high_holding_torque_enable(self):
        """
        Enable high holding torque mode. This increases the robot's stiffness when holding position, providing better positional accuracy under load.
        """
        result = self._send_command("high_holding_torque_enable()")
        
        if result == 0:
            print("[INFO] High holding torque enabled")
        
        return result

    # ------------------------------------------------------------
    # Transformation functions
    # ------------------------------------------------------------
    def pose_trans(self, p_from, p_from_to):
        """
        Pose transformation: transform p_from_to (in frame of p_from) to base frame
        """
        def pose_to_matrix(pose):
            """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
            T = np.eye(4)
            T[0:3, 0:3] = rotvec_to_matrix(pose[3], pose[4], pose[5])
            T[0:3, 3] = [pose[0], pose[1], pose[2]]
            return T
        
        def matrix_to_pose(T):
            """Convert 4x4 homogeneous transformation matrix to pose [X,Y,Z,Rx,Ry,Rz]"""
            x, y, z = T[0:3, 3]
            rotvec = matrix_to_rotvec(T[0:3, 0:3])
            return [x, y, z, rotvec[0], rotvec[1], rotvec[2]]
        
        # Convert poses to transformation matrices
        T_from = pose_to_matrix(p_from)
        T_from_to = pose_to_matrix(p_from_to)
        
        # Multiply matrices: T_result = T_from * T_from_to
        T_result = np.matmul(T_from, T_from_to)
        
        # Convert back to pose
        result = matrix_to_pose(T_result)
        
        return result

