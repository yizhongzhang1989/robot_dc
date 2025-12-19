import os
import json
import numpy as np
import requests
from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import argparse
from robot_status_redis.client_utils import RobotStatusClient
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class UROperateTools(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None):
        # Call parent class constructor (UROperateWobj)
        # Note: server_index is not needed for tools operations, use default
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=0)
        
        # Movement parameters specific to tools operations
        self.a_movej = 0.5  # Acceleration for joint movements (rad/s²)
        self.v_movej = 1.0  # Velocity for joint movements (rad/s)
        self.a_movel = 0.3  # Acceleration for joint movements (m/s²)
        self.v_movel = 0.2  # Velocity for joint movements (m/s)
        
        # Tool coordinate system storage (specific to UROperateTools)
        self.tool_transformation_matrix = None
        self.tool_offset = None
    
    # Note: The following methods are now inherited from UROperateWobj base class:
    # - _load_robot_parameters_from_config()
    # - _setup_paths()
    # - _initialize_robot()
    # - _init_rs485_socket()
    # - _initialize_robot_status_client()
    # - ur_unlock_quick_changer()
    # - ur_lock_quick_changer()
    
    # ================================== Tool Operation Methods ==================================
    def movej_from_task_to_tools(self):
        """
        Move robot joints through predefined waypoints from task position to tools area
        Uses 7 predefined joint positions for safe trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_task_to_tools")
            return False
        
        # Define the 7 waypoints in degrees (convert to radians for robot)
        waypoints_degrees = [
            [85.71, -45.96, 117.38, -63.29, 13.73, 183.24],   # Waypoint 1 (from task position)
            [76.47, -45.96, 140.84, -63.29, 13.73, 183.24],   # Waypoint 2
            [76.47, -122.06, 140.84, -63.29, 13.73, 183.24],  # Waypoint 3
            [76.47, -98.15, 18.49, -63.29, 13.73, 183.24],    # Waypoint 4
            [194.04, -98.15, 18.49, -63.29, 13.73, 183.24],   # Waypoint 5
            [194.04, -93.88, 79.31, -63.29, 13.73, 183.24],   # Waypoint 6
            [194.04, -93.88, 76.55, -71.93, -88.84, 0]   # Waypoint 7 (final tools position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movej_from_task_to_tools trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/7...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movej_from_task_to_tools trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_task_to_tools: {e}")
            return False
    
    def movej_from_tool_to_task(self):
        """
        Move robot joints through predefined waypoints from tools area to task position
        Uses 7 predefined joint positions in reverse order for safe trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_task")
            return False
        
        # Define the 7 waypoints in degrees (reverse order from tools to task)
        waypoints_degrees = [
            [194.04, -93.88, 76.55, -71.93, -88.84, 0],  # Waypoint 1 (from tools position)
            [194.04, -93.88, 79.31, -63.29, 13.73, 183.24],   # Waypoint 2
            [194.04, -98.15, 18.49, -63.29, 13.73, 183.24],   # Waypoint 3
            [76.47, -98.15, 18.49, -63.29, 13.73, 183.24],    # Waypoint 4
            [76.47, -122.06, 140.84, -63.29, 13.73, 183.24],  # Waypoint 5
            [76.47, -45.96, 140.84, -63.29, 13.73, 183.24],   # Waypoint 6
            [85.71, -45.96, 117.38, -63.29, 13.73, 183.24]    # Waypoint 7 (final task position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movel_from_tool_to_task trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/7...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movel_from_tool_to_task trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_task: {e}")
            return False
    
    def movel_from_tool_to_get_tool_pushpull(self):
        """
        Move robot using linear movements to get tool_pushpull from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_pushpull")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.49623, 0.50446, 0.57265, 3.041, -0.842, 0.001],  # Waypoint 1 (approach tool_pushpull)
            [0.49623, 0.50446, 0.39298, 3.041, -0.842, 0.001],  # Waypoint 2 (align with tool_pushpull)
            [0.49623, 0.63619, 0.39298, 3.041, -0.842, 0.001],  # Waypoint 3 (engage tool_pushpull)
            [0.49623, 0.63619, 0.65947, 3.041, -0.842, 0.001]   # Waypoint 4 (secure tool_pushpull)
        ]
        
        print("Starting movel_from_tool_to_get_tool_pushpull trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.ur_lock_quick_changer()
            if lock_result != 0:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_pushpull trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_pushpull: {e}")
            return False
    
    def movel_from_tool_to_return_tool_pushpull(self):
        """
        Move robot using linear movements to return tool_pushpull to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_pushpull")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_pushpull
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.49623, 0.63619, 0.65947, 3.041, -0.842, 0.001],   # Waypoint 1 (from elevated position)
            [0.49623, 0.63619, 0.39298, 3.041, -0.842, 0.001],   # Waypoint 2 (approach release)
            [0.49623, 0.50446, 0.39298, 3.041, -0.842, 0.001],   # Waypoint 3 (release position)
            [0.49623, 0.50446, 0.57265, 3.041, -0.842, 0.001]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_pushpull trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_pushpull...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.ur_unlock_quick_changer()
            if unlock_result != 0:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_pushpull trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_pushpull: {e}")
            return False
    
    def movel_from_tool_to_get_tool_rotate(self):
        """
        Move robot using linear movements to get tool_rotate from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_rotate")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.64116, 0.47126, 0.57795, 1.532, -2.748, 0.009],  # Waypoint 1 (approach tool_rotate)
            [0.64116, 0.47126, 0.38944, 1.532, -2.748, 0.009],  # Waypoint 2 (align with tool_rotate)
            [0.75134, 0.46889, 0.39128, 1.448, -2.777, 0.005],  # Waypoint 3 (engage tool_rotate)
            [0.75134, 0.46889, 0.71440, 1.448, -2.777, 0.005]   # Waypoint 4 (secure tool_rotate)
        ]
        
        print("Starting movel_from_tool_to_get_tool_rotate trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.ur_lock_quick_changer()
            if lock_result != 0:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_rotate trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_rotate: {e}")
            return False
    
    def movel_from_tool_to_return_tool_rotate(self):
        """
        Move robot using linear movements to return tool_rotate to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_rotate")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_rotate
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.75134, 0.46889, 0.71440, 1.448, -2.777, 0.005],   # Waypoint 1 (from elevated position)
            [0.75134, 0.46889, 0.39128, 1.448, -2.777, 0.005],   # Waypoint 2 (approach release)
            [0.64116, 0.47126, 0.38944, 1.532, -2.748, 0.009],   # Waypoint 3 (release position)
            [0.64116, 0.47126, 0.57795, 1.532, -2.748, 0.009]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_rotate trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_rotate...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.ur_unlock_quick_changer()
            if unlock_result != 0:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_rotate trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_rotate: {e}")
            return False
    
    def movel_from_tool_to_get_tool_extract(self):
        """
        Move robot using linear movements to get tool_extract from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_extract")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.64134, 0.38184, 0.50183, 1.466, -2.779, 0.011],  # Waypoint 1 (approach tool_extract)
            [0.64134, 0.38184, 0.39193, 1.466, -2.779, 0.011],  # Waypoint 2 (align with tool_extract)
            [0.77048, 0.38184, 0.39193, 1.466, -2.779, 0.011],  # Waypoint 3 (engage tool_extract)
            [0.77048, 0.38184, 0.65495, 1.466, -2.779, 0.011]   # Waypoint 4 (secure tool_extract)
        ]
        
        print("Starting movel_from_tool_to_get_tool_extract trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.ur_lock_quick_changer()
            if lock_result != 0:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_extract trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_extract: {e}")
            return False
    
    def movel_from_tool_to_return_tool_extract(self):
        """
        Move robot using linear movements to return tool_extract to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_extract")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_extract
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.77048, 0.38184, 0.65495, 1.466, -2.779, 0.011],   # Waypoint 1 (from elevated position)
            [0.77048, 0.38184, 0.39193, 1.466, -2.779, 0.011],   # Waypoint 2 (approach release)
            [0.64134, 0.38184, 0.39193, 1.466, -2.779, 0.011],   # Waypoint 3 (release position)
            [0.64134, 0.38184, 0.50183, 1.466, -2.779, 0.011]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_extract trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_extract...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.ur_unlock_quick_changer()
            if unlock_result != 0:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_extract trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_extract: {e}")
            return False
    
    def get_tool_from_task_position(self, tool_name):
        """
        Complete workflow to get a tool from task position
        Executes: task->tools->get_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting get_tool_from_task_position workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool_name not in valid_tools:
            print(f"✗ Invalid tool name '{tool_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Get the specified tool
            print(f"Step 2: Getting {tool_name}...")
            get_method_name = f"movel_from_tool_to_get_{tool_name}"
            if hasattr(self, get_method_name):
                get_method = getattr(self, get_method_name)
                if not get_method():
                    print(f"✗ Failed to get {tool_name}")
                    return False
            else:
                print(f"✗ Method {get_method_name} not found")
                return False
            
            # Step 3: Return to task position with tool
            print("Step 3: Returning to task position with tool...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed get_tool_from_task_position workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during get_tool_from_task_position: {e}")
            return False
    
    def return_tool_from_task_position(self, tool_name):
        """
        Complete workflow to return a tool from task position
        Executes: task->tools->return_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting return_tool_from_task_position workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool_name not in valid_tools:
            print(f"✗ Invalid tool name '{tool_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Return the specified tool
            print(f"Step 2: Returning {tool_name}...")
            return_method_name = f"movel_from_tool_to_return_{tool_name}"
            if hasattr(self, return_method_name):
                return_method = getattr(self, return_method_name)
                if not return_method():
                    print(f"✗ Failed to return {tool_name}")
                    return False
            else:
                print(f"✗ Method {return_method_name} not found")
                return False
            
            # Step 3: Return to task position without tool
            print("Step 3: Returning to task position without tool...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed return_tool_from_task_position workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool_from_task_position: {e}")
            return False

    def return_tool1_get_tool2_from_task(self, tool1_name, tool2_name):
        """
        Complete workflow to return one tool and get another tool from task position
        Executes: task->tools->return_tool1->get_tool2->task
        
        Args:
            tool1_name (str): Name of the tool to return ('tool_pushpull', 'tool_rotate', or 'tool_extract')
            tool2_name (str): Name of the tool to get ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting return_tool1_get_tool2_from_task workflow: returning {tool1_name}, getting {tool2_name}...")
        
        # Validate tool names
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool1_name not in valid_tools:
            print(f"✗ Invalid tool1 name '{tool1_name}'. Valid tools: {valid_tools}")
            return False
        if tool2_name not in valid_tools:
            print(f"✗ Invalid tool2 name '{tool2_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Return tool1
            print(f"Step 2: Returning {tool1_name}...")
            return_method_name = f"movel_from_tool_to_return_{tool1_name}"
            if hasattr(self, return_method_name):
                return_method = getattr(self, return_method_name)
                if not return_method():
                    print(f"✗ Failed to return {tool1_name}")
                    return False
            else:
                print(f"✗ Method {return_method_name} not found")
                return False
            
            # Step 3: Get tool2
            print(f"Step 3: Getting {tool2_name}...")
            get_method_name = f"movel_from_tool_to_get_{tool2_name}"
            if hasattr(self, get_method_name):
                get_method = getattr(self, get_method_name)
                if not get_method():
                    print(f"✗ Failed to get {tool2_name}")
                    return False
            else:
                print(f"✗ Method {get_method_name} not found")
                return False
            
            # Step 4: Return to task position with tool2
            print("Step 4: Returning to task position with tool2...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed return_tool1_get_tool2_from_task workflow: returned {tool1_name}, got {tool2_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool1_get_tool2_from_task: {e}")
            return False

# ===================================== Frame Operation Functions ===================================== 
    def movej_from_task_to_frame(self):
        """
        Move robot joints through predefined waypoints from task position to frame position
        Uses 5 predefined joint positions for trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_task_to_frame")
            return False
        
        # Define the 5 waypoints in degrees (convert to radians for robot)
        waypoints_degrees = [
            [85.8, -72.7, 107.9, -26.1, 13.5, 182.3],   # Waypoint 1
            [76.4, -121.1, 140.8, -63.3, 13.7, 183.5],  # Waypoint 2
            [76.5, -98.1, 18.5, -63.3, 13.7, 183.5],    # Waypoint 3
            [221.0, -98.1, 18.5, -63.3, 13.7, 183.5],   # Waypoint 4
            [235.4, -82.3, 125.7, -133.3, -90.0, -2.4]  # Waypoint 5 (final frame position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movej_from_task_to_frame trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/5...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movej_from_task_to_frame trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_task_to_frame: {e}")
            return False
    
    def movej_from_frame_to_task(self):
        """
        Move robot joints through predefined waypoints from frame position to task position
        Uses 4 predefined joint positions for trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_frame_to_task")
            return False
        
        # Define the 5 waypoints in degrees (convert to radians for robot)
        waypoints_degrees = [
            [241.0, -90.9, 1.0, 1.9, -90.0, 3.6],       # Waypoint 1 (from frame position)
            [241.0, -90.9, 1.0, -178.6, -90.0, 3.6],    # Waypoint 2
            [87.0, -90.9, 1.0, -178.6, -90.0, 3.6],     # Waypoint 3
            [87.3, -83.5, 71.4, -76.0, -93.4, -53.9],    # Waypoint 4 
            [92.4, -59.6, 92.8, -30.3, 10.6, 177.7]    # Waypoint 5 (final task position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movej_from_frame_to_task trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/5...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movej_from_frame_to_task trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_frame_to_task: {e}")
            return False
        
    def movel_from_frame_to_get_tool_frame(self):
        """
        Move robot using linear movements to get tool_frame from frame position
        Uses 3 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        Executes: position 1 -> lock_command -> position 2 -> position 3
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_frame_to_get_tool_frame")
            return False
        
        # Define the 3 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.19354, 0.59381, 0.14359, 3.019, -0.879, 0.009],  # Waypoint 1 (approach frame)
            [0.19354, 0.73082, 0.14430, 3.019, -0.879, 0.009],  # Waypoint 2 (after lock, engage frame)
            [0.19354, 0.73082, 0.77569, 3.019, -0.879, 0.009]   # Waypoint 3 (secure frame)
        ]
        
        print("Starting movel_from_frame_to_get_tool_frame trajectory with movel...")
        
        try:
            # Execute first waypoint
            print("Executing first phase: approach frame position...")
            waypoint = waypoints_cartesian[0]
            print(f"Moving to waypoint 1/3 in getting tool_frame...")
            
            # Execute linear movement
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 1, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 1")
            
            # Add small delay between movements for stability
            time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command
            lock_result = self.ur_lock_quick_changer()
            if lock_result != 0:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute remaining waypoints
            print("Executing second phase: engage and secure frame...")
            for i, waypoint in enumerate(waypoints_cartesian[1:], 2):
                print(f"Moving to waypoint {i}/3 in getting tool_frame...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_frame_to_get_tool_frame trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_frame_to_get_tool_frame: {e}")
            return False
    
    def get_frame_from_task_position(self):
        """
        Complete workflow to get frame from task position
        Executes: task->frame->get_frame->task
        
        Returns:
            bool: True if successful, False otherwise
        """
        print("Starting get_frame_from_task_position workflow...")
        
        if not self.robot:
            print("✗ Robot not connected, cannot execute get_frame_from_task_position")
            return False
        
        try:
            # Step 1: Move from task position to frame area
            print("Step 1: Moving from task position to frame area...")
            if not self.movej_from_task_to_frame():
                print("✗ Failed to move from task to frame area")
                return False
            
            # Step 2: Get frame
            print("Step 2: Getting frame...")
            if not self.movel_from_frame_to_get_tool_frame():
                print("✗ Failed to get frame")
                return False
            
            # Step 3: Return to task position with frame
            print("Step 3: Returning to task position with frame...")
            if not self.movej_from_frame_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print("✓ Successfully completed get_frame_from_task_position workflow")
            return True
            
        except Exception as e:
            print(f"✗ Exception during get_frame_from_task_position: {e}")
            return False
    
    def return_tool_get_frame_from_task(self, tool_name):
        """
        Complete workflow to return a tool and get frame tool from task position
        Executes: task->tools->return_tool->get_frame_tool->task
        
        Args:
            tool_name (str): Name of the tool to return ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting return_tool_get_frame_from_task workflow: returning {tool_name}, getting frame tool...")
        
        # Validate tool name
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool_name not in valid_tools:
            print(f"✗ Invalid tool name '{tool_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Return the specified tool
            print(f"Step 2: Returning {tool_name}...")
            return_method_name = f"movel_from_tool_to_return_{tool_name}"
            if hasattr(self, return_method_name):
                return_method = getattr(self, return_method_name)
                if not return_method():
                    print(f"✗ Failed to return {tool_name}")
                    return False
            else:
                print(f"✗ Method {return_method_name} not found")
                return False
            
            # Step 3: Move to intermediate waypoint 1
            print("Step 3: Moving to intermediate waypoint 1...")
            waypoint1_degrees = [205.1, -86.8, 86.6, -90.6, -89.7, 34.3]
            waypoint1_radians = [np.deg2rad(angle) for angle in waypoint1_degrees]
            move_result = self.robot.movej(waypoint1_radians, a=self.a_movej, v=self.v_movej)
            time.sleep(0.5)
            if move_result:
                print("✗ Failed to move to intermediate waypoint 1")
                return False
            print("✓ Reached intermediate waypoint 1")
            
            # Step 4: Move to intermediate waypoint 2
            print("Step 4: Moving to intermediate waypoint 2...")
            waypoint2_degrees = [233.6, -80.7, 92.0, -107.2, -90.0, -3.5]
            waypoint2_radians = [np.deg2rad(angle) for angle in waypoint2_degrees]
            move_result = self.robot.movej(waypoint2_radians, a=self.a_movej, v=self.v_movej)
            time.sleep(0.5)
            if move_result:
                print("✗ Failed to move to intermediate waypoint 2")
                return False
            print("✓ Reached intermediate waypoint 2")
            
            # Step 5: Move to intermediate waypoint 3
            print("Step 5: Moving to intermediate waypoint 3...")
            waypoint3_degrees = [235.4, -82.3, 125.7, -133.3, -90.0, -2.4]
            waypoint3_radians = [np.deg2rad(angle) for angle in waypoint3_degrees]
            move_result = self.robot.movej(waypoint3_radians, a=self.a_movej, v=self.v_movej)
            time.sleep(0.5)
            if move_result:
                print("✗ Failed to move to intermediate waypoint 3")
                return False
            print("✓ Reached intermediate waypoint 3")

            # Step 6: Get frame tool
            print("Step 6: Getting frame tool...")
            if not self.movel_from_frame_to_get_tool_frame():
                print("✗ Failed to get frame tool")
                return False
            
            # Step 7: Return to task position with frame tool
            print("Step 7: Returning to task position with frame tool...")
            if not self.movej_from_frame_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed return_tool_get_frame_from_task workflow: returned {tool_name}, got frame tool")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool_get_frame_from_task: {e}")
            return False

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UR15 Robot Tool Operation')
    parser.add_argument('--robot_ip', type=str, default=None, help='Robot IP address')
    parser.add_argument('--robot_port', type=int, default=None, help='Robot port number')
    
    args = parser.parse_args()
    
    # Initialize UROperateTools
    ur_tools = UROperateTools(robot_ip=args.robot_ip, robot_port=args.robot_port)

    print("UROperateTools initialized successfully")
    print("Ready for tool operations...")
    ur_tools.get_frame_from_task_position()