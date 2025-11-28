#!/usr/bin/env python3
"""
Robot Movement Handler
"""

import sys
import time
from pathlib import Path
from typing import Dict, Any

# Add scripts directory to path for robot imports
sys.path.append(str(Path(__file__).parent.parent.parent / 'scripts'))

from ur15_workflow.base import OperationHandler

try:
    from ur15_robot_arm.ur15 import UR15Robot
except ImportError:
    print("Warning: UR15Robot not available")
    UR15Robot = None


class RobotMoveHandler(OperationHandler):
    """
    Handler for robot movement operations
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute robot movement
        
        Operation parameters:
            - robot_pose: Target pose [x,y,z,rx,ry,rz] or reference to load
            - move_type: 'movej' or 'movel' (default: 'movej')
            - velocity: Movement velocity (default: 0.5)
            - acceleration: Movement acceleration (default: 0.5)
            - wait_time: Time to wait after movement (default: 1.0)
        """
        try:
            if UR15Robot is None:
                return {'status': 'error', 'error': 'UR15Robot not available'}
            
            # Get robot from context or create new instance
            robot = context.get('robot')
            if robot is None:
                robot_ip = context.get('robot_ip', '192.168.1.15')
                robot_port = context.get('robot_port', 30002)
                robot = UR15Robot(robot_ip, robot_port)
                if robot.open() != 0:
                    return {'status': 'error', 'error': f'Failed to connect to robot at {robot_ip}:{robot_port}'}
                context['robot'] = robot
            
            # Resolve target pose
            target_pose = self._resolve_parameter(operation.get('robot_pose'), context)
            
            if target_pose is None:
                return {'status': 'error', 'error': 'No target pose specified'}
            
            # Convert to list if needed
            if isinstance(target_pose, str):
                # Try to parse as comma-separated values
                target_pose = [float(x) for x in target_pose.split(',')]
            
            # Get movement parameters
            move_type = operation.get('move_type', 'movej')
            velocity = operation.get('velocity', 0.5)
            acceleration = operation.get('acceleration', 0.5)
            wait_time = operation.get('wait_time', 1.0)
            
            print(f"    Moving robot to pose: {target_pose}")
            print(f"    Move type: {move_type}, v={velocity}, a={acceleration}")
            
            # Execute movement
            if move_type == 'movej':
                result_code = robot.movej(target_pose, a=acceleration, v=velocity, t=0, r=0)
            elif move_type == 'movel':
                result_code = robot.movel(target_pose, a=acceleration, v=velocity, t=0, r=0)
            elif move_type == 'move_tcp':
                # For move_tcp, target_pose is treated as the offset [x,y,z,rx,ry,rz]
                result_code = robot.move_tcp(target_pose, a=acceleration, v=velocity, t=0, r=0)
            else:
                return {'status': 'error', 'error': f"Unknown move type: {move_type}"}
            
            if result_code != 0:
                return {
                    'status': 'error',
                    'error': f'Robot movement failed with code {result_code}'
                }
            
            # Wait for stabilization
            time.sleep(wait_time)
            
            # Read current pose
            current_pose = robot.get_actual_tcp_pose()
            
            return {
                'status': 'success',
                'outputs': {
                    'current_robot_pose': current_pose
                }
            }
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
