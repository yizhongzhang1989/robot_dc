#!/usr/bin/env python3
"""
3D Positioning Handler
"""

import os
import cv2
import json
import numpy as np
from datetime import datetime
from typing import Dict, Any, Tuple
from ur15_workflow.base import OperationHandler


class PositioningHandler(OperationHandler):
    """
    Handler for 3D positioning operations
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute 3D positioning operations
        
        Operation parameters:
            - positioning_type: Type of operation (upload_references, init_session, upload_view, get_result, terminate_session)
            - ... specific parameters for each type
        """
        try:
            # Get positioning client from context
            positioning_client = context.get('positioning_client')
            
            if positioning_client is None:
                return {'status': 'error', 'error': 'Positioning client not initialized'}
            
            positioning_type = operation.get('positioning_type')
            if not positioning_type:
                return {'status': 'error', 'error': 'positioning_type is required'}
            
            if positioning_type == 'upload_references':
                return self._handle_upload_references(positioning_client)
            elif positioning_type == 'init_session':
                return self._handle_init_session(positioning_client, context)
            elif positioning_type == 'upload_view':
                return self._handle_upload_view(positioning_client, operation, context, previous_results)
            elif positioning_type == 'get_result':
                return self._handle_get_result(positioning_client, operation, context)
            elif positioning_type == 'terminate_session':
                return self._handle_terminate_session(positioning_client, context)
            else:
                return {'status': 'error', 'error': f'Unknown positioning_type: {positioning_type}'}
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            return {'status': 'error', 'error': str(e)}

    def _handle_upload_references(self, client) -> Dict[str, Any]:
        print("    Uploading references...")
        result = client.upload_references()
        if result.get('success'):
            print(f"    ✓ References uploaded: {result.get('references_loaded')}/{result.get('references_found')}")
            return {'status': 'success', 'outputs': result}
        else:
            return {'status': 'error', 'error': result.get('error', 'Failed to upload references')}

    def _handle_init_session(self, client, context: Dict[str, Any]) -> Dict[str, Any]:
        print("    Initializing session...")
        # Check if session already exists? Maybe we want to force new one.
        # For now, just create new one.
        result = client.init_session()
        if result.get('success'):
            session_id = result['session_id']
            context['current_session_id'] = session_id
            print(f"    ✓ Session created: {session_id}")
            return {'status': 'success', 'outputs': {'session_id': session_id}}
        else:
            return {'status': 'error', 'error': result.get('error', 'Failed to init session')}

    def _handle_upload_view(self, client, operation: Dict[str, Any], context: Dict[str, Any], 
                           previous_results: Dict[str, Any]) -> Dict[str, Any]:
        session_id = context.get('current_session_id')
        if not session_id:
            # Try to auto-create session if missing?
            # For now, fail if no session.
            # Or maybe we can support creating one if requested.
            return {'status': 'error', 'error': 'No active session found in context'}
            
        reference_name = operation.get('reference_name')
        if not reference_name:
            return {'status': 'error', 'error': 'reference_name is required for upload_view'}
            
        # Resolve image and pose paths
        # They might be in previous_results from 'capture_image' and 'record_data' steps
        # We need to know the keys used in previous steps.
        # Assuming standard keys or passed via operation config.
        
        image_path = self._resolve_parameter(operation.get('image_path'), context)
        pose_path = self._resolve_parameter(operation.get('pose_path'), context)
        
        # If not explicitly provided, try to find in previous results
        if not image_path:
            # Look for last capture result
            # This is tricky because previous_results is a dict of all previous steps.
            # We might need to look for specific keys.
            # For now, let's assume the user passes them or we find them in specific keys.
            pass
            
        if not image_path or not os.path.exists(image_path):
            return {'status': 'error', 'error': f'Image path not found: {image_path}'}
            
        if not pose_path or not os.path.exists(pose_path):
            return {'status': 'error', 'error': f'Pose path not found: {pose_path}'}
            
        print(f"    Uploading view for {reference_name}...")
        
        try:
            # Load image
            image = cv2.imread(image_path)
            if image is None:
                return {'status': 'error', 'error': f'Failed to load image: {image_path}'}
                
            # Load params
            intrinsic, distortion, extrinsic = self._load_camera_params(pose_path, context)
            
            # Upload
            result = client.upload_view(
                session_id=session_id,
                reference_name=reference_name,
                image=image,
                intrinsic=intrinsic,
                distortion=distortion,
                extrinsic=extrinsic
            )
            
            if result.get('success'):
                print(f"    ✓ View uploaded successfully")
                return {'status': 'success', 'outputs': result}
            else:
                return {'status': 'error', 'error': result.get('error', 'Failed to upload view')}
                
        except Exception as e:
            return {'status': 'error', 'error': f'Error processing view: {str(e)}'}

    def _handle_get_result(self, client, operation: Dict[str, Any], context: Dict[str, Any]) -> Dict[str, Any]:
        session_id = context.get('current_session_id')
        if not session_id:
            return {'status': 'error', 'error': 'No active session'}
            
        # Get template_points - can be None, empty list, or list of points
        template_points = operation.get('template_points')
        timeout = operation.get('timeout', 10000)
        
        # Get robot_status save parameters
        status_namespace = operation.get('status_namespace')  # e.g., "ur15"
        status_key_name = operation.get('status_key_name')  # e.g., "rack_points_3d"
        local2world_matrix_name = operation.get('local2world_matrix_name')  # e.g., "rack_local2world"
        
        # Determine positioning mode
        mode = self._determine_positioning_mode(template_points)
        
        if mode == 'fitting':
            print(f"    Computing 3D positions (Fitting mode) for session {session_id}...")
            print(f"    → Using {len(template_points)} template points with full 3D coordinates")
        elif mode == 'filtered':
            print(f"    Computing 3D positions (Filtered triangulation) for session {session_id}...")
            print(f"    → Filtering by {len(template_points)} point names")
        else:  # standard
            print(f"    Computing 3D positions (Standard triangulation) for session {session_id}...")
            print(f"    → Using all tracked keypoints")
        
        result = client.get_result(
            session_id,
            template_points=template_points,
            timeout=timeout
        )
        
        if result.get('success'):
            positioning_result = result.get('result', {})
            points_3d = positioning_result.get('points_3d', [])
            mean_error = positioning_result.get('mean_error', 0)
            print(f"    ✓ Positioning completed: {len(points_3d)} points, error={mean_error:.2f}px")
            
            # Save result to file if save_path is provided
            save_path = self._resolve_parameter(operation.get('save_path'), context)
            if save_path:
                try:
                    os.makedirs(os.path.dirname(save_path), exist_ok=True)
                    
                    # Prepare data to save
                    save_data = {
                        'timestamp': datetime.now().isoformat(),
                        'session_id': session_id,
                        'points_3d': points_3d,
                        'mean_error': mean_error,
                        'positioning_result': positioning_result,
                        'template_points': template_points
                    }
                    
                    with open(save_path, 'w') as f:
                        json.dump(save_data, f, indent=2)
                    print(f"    ✓ Result saved to {save_path}")
                except Exception as e:
                    print(f"    ⚠ Warning: Failed to save result to {save_path}: {e}")

            # Save to robot_status if configured
            if status_namespace and (status_key_name or local2world_matrix_name):
                robot_status_client = context.get('robot_status_client')
                if robot_status_client:
                    try:
                        print(f"    📤 Saving results to robot_status (namespace: {status_namespace})...")
                        
                        # Save 3D points if key name is provided
                        if status_key_name:
                            if points_3d:
                                if robot_status_client.set_status(status_namespace, status_key_name, points_3d):
                                    print(f"    ✓ {status_key_name} saved to robot_status")
                                else:
                                    print(f"    ✗ Failed to save {status_key_name}")
                            else:
                                print(f"    ⚠ No 3D points to save")
                        
                        # Save local2world matrix if in fitting mode and key name is provided
                        if mode == 'fitting' and local2world_matrix_name:
                            local2world = positioning_result.get('local2world')
                            if local2world is not None:
                                # Convert to list if it's numpy array
                                if hasattr(local2world, 'tolist'):
                                    local2world = local2world.tolist()
                                
                                if robot_status_client.set_status(status_namespace, local2world_matrix_name, local2world):
                                    print(f"    ✓ {local2world_matrix_name} saved to robot_status (4x4 transformation matrix)")
                                else:
                                    print(f"    ✗ Failed to save {local2world_matrix_name}")
                            else:
                                print(f"    ⚠ No local2world matrix in result (fitting mode expected)")
                        elif local2world_matrix_name and mode != 'fitting':
                            print(f"    ⚠ local2world_matrix_name specified but not in fitting mode (current: {mode})")
                            
                    except Exception as e:
                        print(f"    ✗ Error saving to robot_status: {e}")
                else:
                    print(f"    ⚠ RobotStatusClient not available, skipping robot_status save")
            
            return {
                'status': 'success',
                'outputs': {
                    'points_3d': points_3d,
                    'mean_error': mean_error,
                    'positioning_result': positioning_result
                }
            }
        else:
            return {'status': 'error', 'error': result.get('error', 'Positioning failed')}

    def _handle_terminate_session(self, client, context: Dict[str, Any]) -> Dict[str, Any]:
        session_id = context.get('current_session_id')
        if not session_id:
            print("    Warning: No session to terminate")
            return {'status': 'success', 'outputs': {'message': 'No session to terminate'}}
            
        print(f"    Terminating session {session_id}...")
        client.terminate_session(session_id)
        
        # Clear session from context
        del context['current_session_id']
        
        print("    ✓ Session terminated")
        return {'status': 'success'}

    def _determine_positioning_mode(self, template_points) -> str:
        """
        Determine positioning mode based on template_points parameter.
        
        Returns:
            'fitting': All points have x,y,z coordinates (fitting mode)
            'filtered': Some/all points only have 'name' (filtered triangulation)
            'standard': template_points is None or empty (standard triangulation)
        """
        if not template_points:
            return 'standard'
        
        # Check if all points have x,y,z coordinates
        all_have_xyz = all(
            isinstance(pt, dict) and 'x' in pt and 'y' in pt and 'z' in pt 
            for pt in template_points
        )
        
        if all_have_xyz:
            return 'fitting'
        else:
            return 'filtered'

    def _load_camera_params(self, json_path: str, context: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Load camera parameters.
        Static params from RobotStatus (ur15 namespace).
        Dynamic params (end2base) from JSON file.
        """
        # Load end2base from JSON
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        end2base = np.array(data['end2base'], dtype=np.float64)
        
        # Get RobotStatusClient
        client = context.get('robot_status_client')
        if not client:
            raise RuntimeError("RobotStatusClient not available in context")
            
        # Load static parameters from robot status
        camera_matrix = client.get_status("ur15", "camera_matrix")
        distortion_coefficients = client.get_status("ur15", "distortion_coefficients")
        cam2end_matrix = client.get_status("ur15", "cam2end_matrix")
        
        if camera_matrix is None:
            raise ValueError("camera_matrix not found in robot status (ur15)")
        if distortion_coefficients is None:
            raise ValueError("distortion_coefficients not found in robot status (ur15)")
        if cam2end_matrix is None:
            raise ValueError("cam2end_matrix not found in robot status (ur15)")
            
        intrinsic = np.array(camera_matrix, dtype=np.float64)
        distortion = np.array(distortion_coefficients, dtype=np.float64)
        cam2end = np.array(cam2end_matrix, dtype=np.float64)
        
        # Calculate cam2base (camera to world/base)
        cam2base = end2base @ cam2end
        
        # Triangulation expects world2cam (world to camera), so invert
        extrinsic = np.linalg.inv(cam2base)
        
        return intrinsic, distortion, extrinsic

