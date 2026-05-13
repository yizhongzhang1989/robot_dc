#!/usr/bin/env python3
"""
Robot Movement Handler
"""

import sys
import time
from pathlib import Path
from typing import Dict, Any
from common.workspace_utils import get_workspace_root, get_scripts_directory

# Add scripts directory to path for robot imports
scripts_dir = get_scripts_directory()
if scripts_dir:
    sys.path.append(scripts_dir)

from ur_workflow.base import OperationHandler

try:
    from ur_robot_arm.ur15 import UR15Robot
except ImportError:
    print("Warning: UR15Robot not available")
    UR15Robot = None


class RobotMoveHandler(OperationHandler):
    """
    Handler for robot movement operations
    """
    
    @staticmethod
    def _resolve_robot_endpoint(context: Dict[str, Any]):
        """Resolve (ip, port, source) for the robot this workflow targets.
        
        Resolution order:
        
        1. ``status_namespace`` from the workflow's context → look up
           ``robot_ip``/``robot_port`` published in robot_status_redis at
           launch time by each web node. This is the preferred path because
           it lets workflows be namespace-only (e.g.
           ``{"context": {"status_namespace": "ur10e"}}``) and stay correct
           if a robot's IP changes in robot_config.yaml without re-editing
           any JSON.
        2. ``robot_ip``/``robot_port`` set directly in the workflow's
           ``context`` block — legacy form, still honored.
        3. Hardcoded ``192.168.1.15:30002`` as a last-resort default to
           preserve the previous behavior for very old workflow JSONs.
        
        Returns ``(ip, port, source)`` where ``source`` is one of
        ``'status_redis'``, ``'context'``, ``'fallback'``.
        """
        # Try robot_status_redis first.
        namespace = context.get('status_namespace')
        client = context.get('robot_status_client')
        if namespace and client is not None:
            try:
                redis_ip = client.get_status(namespace, 'robot_ip')
                redis_port = client.get_status(namespace, 'robot_port')
            except Exception as exc:  # noqa: BLE001
                print(f"    ⚠ Failed to read robot endpoint from "
                      f"robot_status[{namespace}]: {exc}")
                redis_ip = None
                redis_port = None
            if redis_ip:
                port = int(redis_port) if redis_port else 30002
                return redis_ip, port, 'status_redis'
        
        # Fall back to the legacy per-workflow context values, then the
        # ur15 hardcoded defaults so we never lose backward compatibility.
        ctx_ip = context.get('robot_ip')
        ctx_port = context.get('robot_port')
        if ctx_ip:
            return ctx_ip, int(ctx_port) if ctx_port else 30002, 'context'
        return '192.168.1.15', 30002, 'fallback'
    
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
                robot_ip, robot_port, source = self._resolve_robot_endpoint(context)
                print(f"    Connecting to robot at {robot_ip}:{robot_port} (source: {source})")
                robot = UR15Robot(robot_ip, robot_port)
                if robot.open() != 0:
                    return {'status': 'error',
                            'error': f'Failed to connect to robot at {robot_ip}:{robot_port}'}
                context['robot'] = robot
                # Stash the resolved endpoint so subsequent handlers (and
                # error messages) see what we actually connected to.
                context['robot_ip'] = robot_ip
                context['robot_port'] = robot_port
            
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
