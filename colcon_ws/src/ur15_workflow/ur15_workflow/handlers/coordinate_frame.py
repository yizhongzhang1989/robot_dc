#!/usr/bin/env python3
"""
Coordinate Frame Building Handler
"""

import numpy as np
from typing import Dict, Any, List
from ur15_workflow.base import OperationHandler


class CoordinateFrameHandler(OperationHandler):
    """
    Handler for building coordinate frames from points
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Build coordinate frame from 3D points
        
        Operation parameters:
            - method: 'four_point_orthogonal', 'plane_fit', etc.
            - point_indices: Which points to use for frame building
        """
        try:
            points_3d = context.get('points_3d')
            
            if points_3d is None:
                return {'status': 'error', 'error': 'No 3D points in context'}
            
            method = operation.get('method', 'four_point_orthogonal')
            
            print(f"    Building coordinate frame using {method}")
            
            # Build frame based on method
            if method == 'four_point_orthogonal':
                frame = self._build_four_point_frame(points_3d)
            else:
                return {'status': 'error', 'error': f'Unknown method: {method}'}
            
            if frame is None:
                return {'status': 'error', 'error': 'Frame building failed'}
            
            print("    ✓ Coordinate frame built successfully")
            
            return {
                'status': 'success',
                'outputs': {
                    'coordinate_frame': frame
                }
            }
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
    
    def _build_four_point_frame(self, points_3d: List) -> Dict[str, Any]:
        """
        Build orthogonal frame from 4 corner points
        Assumes points are: [bottom_left, bottom_right, top_left, top_right]
        """
        points = np.array(points_3d)
        
        if len(points) < 4:
            return None
        
        # Origin at bottom left
        origin = points[0]
        
        # X-axis: bottom_left → bottom_right
        x_vec = points[1] - points[0]
        x_vec = x_vec / np.linalg.norm(x_vec)
        
        # Z-axis: bottom_left → top_left
        z_vec = points[2] - points[0]
        z_vec = z_vec / np.linalg.norm(z_vec)
        
        # Y-axis: Z × X (right-hand rule)
        y_vec = np.cross(z_vec, x_vec)
        y_vec = y_vec / np.linalg.norm(y_vec)
        
        # Re-orthogonalize Z
        z_vec = np.cross(x_vec, y_vec)
        z_vec = z_vec / np.linalg.norm(z_vec)
        
        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x_vec, y_vec, z_vec])
        T[:3, 3] = origin
        
        return {
            'origin': origin.tolist(),
            'x_axis': x_vec.tolist(),
            'y_axis': y_vec.tolist(),
            'z_axis': z_vec.tolist(),
            'transformation_matrix': T.tolist()
        }
