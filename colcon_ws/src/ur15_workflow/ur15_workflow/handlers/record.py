#!/usr/bin/env python3
"""
Record Data Handler
"""

import os
import json
import time
import numpy as np
from datetime import datetime
from typing import Dict, Any
from scipy.spatial.transform import Rotation as R
from ur15_workflow.base import OperationHandler

try:
    from ur15_robot_arm.ur15 import UR15Robot
except ImportError:
    print("Warning: UR15Robot not available")
    UR15Robot = None


class RecordDataHandler(OperationHandler):
    """
    Handler for recording robot data (pose, joints, etc.)
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Record robot data to memory
        
        Operation parameters:
            - record_type: Type of data to record (default: 'pose')
            - pose_name: Name to identify this pose data in context (e.g., "1.json")
        """
        try:
            if UR15Robot is None:
                return {'status': 'error', 'error': 'UR15Robot not available'}
            
            # Get robot from context
            robot = context.get('robot')
            if robot is None:
                return {'status': 'error', 'error': 'Robot not initialized in context'}
            
            # Get parameters
            record_type = operation.get('record_type', 'pose')
            pose_name = self._resolve_parameter(operation.get('pose_name'), context)
            
            # Auto-generate pose name if not specified
            if not pose_name:
                # Try to get image name from previous operation (if it was a capture)
                latest_image_name = None
                for op_id, result in reversed(list(previous_results.items())):
                    if result.get('status') == 'success' and 'image_name' in result:
                        latest_image_name = result['image_name']
                        break
                
                if latest_image_name:
                    # Use same name as latest captured image, but change extension to .json
                    import os
                    base_name = os.path.splitext(latest_image_name)[0]  # Remove extension
                    pose_name = f"{base_name}.json"
                    print(f"    Using latest image name for pose: {pose_name}")
                else:
                    # Fall back to timestamp
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    pose_name = f"{timestamp}.json"
                    print(f"    Auto-generated pose name: {pose_name}")
            
            print(f"    Recording {record_type} data as '{pose_name}'...")
            
            data = {}
            
            if record_type == 'pose':
                # Read robot data
                joint_positions = robot.get_actual_joint_positions()
                if joint_positions is None:
                    return {'status': 'error', 'error': 'Failed to read joint positions'}
                    
                tcp_pose = robot.get_actual_tcp_pose()
                if tcp_pose is None:
                    return {'status': 'error', 'error': 'Failed to read TCP pose'}
                
                # Calculate end2base transformation matrix
                end2base = np.eye(4)
                end2base[:3, :3] = R.from_rotvec(tcp_pose[3:6]).as_matrix()
                end2base[:3, 3] = tcp_pose[:3]
                
                # Load camera parameters if provided
                camera_params = {
                    'camera_matrix': None,
                    'distortion_coefficients': None,
                    'cam2end_matrix': None
                }
                
                # Load camera parameters
                camera_params = self._load_camera_parameters(context)
                
                # Prepare data structure (matching ur_capture.py format)
                data = {
                    "joint_angles": list(joint_positions),
                    "end_xyzrpy": {
                        "x": tcp_pose[0],
                        "y": tcp_pose[1],
                        "z": tcp_pose[2],
                        "rx": tcp_pose[3],
                        "ry": tcp_pose[4],
                        "rz": tcp_pose[5]
                    },
                    "end2base": end2base.tolist(),
                    "camera_matrix": camera_params['camera_matrix'],
                    "distortion_coefficients": camera_params['distortion_coefficients'],
                    "cam2end_matrix": camera_params['cam2end_matrix'],
                    "timestamp": datetime.now().isoformat()
                }
            else:
                return {'status': 'error', 'error': f"Unknown record_type: {record_type}"}
            
            print(f"    ✓ Pose data stored in memory as '{pose_name}'")
            
            # Store data in context with numpy arrays for efficient access
            return {
                'status': 'success',
                'pose_name': pose_name,  # Return pose name for next operation
                'outputs': {
                    pose_name: {
                        'joint_angles': np.array(data['joint_angles']),
                        'end_xyzrpy': data['end_xyzrpy'],
                        'end2base': np.array(data['end2base']),
                        'camera_matrix': np.array(data['camera_matrix']) if data['camera_matrix'] else None,
                        'distortion_coefficients': np.array(data['distortion_coefficients']) if data['distortion_coefficients'] else None,
                        'cam2end_matrix': np.array(data['cam2end_matrix']) if data['cam2end_matrix'] else None,
                        'timestamp': data['timestamp']
                    }
                }
            }
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
    
    def _load_camera_parameters(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Load camera parameters from RobotStatusClient
        """
        camera_params = {
            'camera_matrix': None,
            'distortion_coefficients': None,
            'cam2end_matrix': None
        }
        
        # Try RobotStatusClient
        client = context.get('robot_status_client')
        if client:
            try:
                # Try to get parameters from status service
                cm = client.get_status("ur15", "camera_matrix")
                dc = client.get_status("ur15", "distortion_coefficients")
                c2e = client.get_status("ur15", "cam2end_matrix")
                
                # Convert numpy arrays to lists for JSON serialization
                if cm is not None: 
                    camera_params['camera_matrix'] = cm.tolist() if hasattr(cm, 'tolist') else cm
                if dc is not None: 
                    camera_params['distortion_coefficients'] = dc.tolist() if hasattr(dc, 'tolist') else dc
                if c2e is not None: 
                    camera_params['cam2end_matrix'] = c2e.tolist() if hasattr(c2e, 'tolist') else c2e
                
            except Exception as e:
                print(f"    ⚠ Warning: Failed to load from RobotStatusClient: {e}")

        return camera_params
