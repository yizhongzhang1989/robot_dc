#!/usr/bin/env python3
"""
3D Positioning Handler
"""

from typing import Dict, Any
from ur15_workflow.base import OperationHandler


class PositioningHandler(OperationHandler):
    """
    Handler for 3D positioning operations
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute 3D positioning using captured images
        
        Operation parameters:
            - session_id: Session ID from positioning service
            - template_points: Template points for fitting
            - timeout: Timeout in seconds
        """
        try:
            # Get positioning client from context
            positioning_client = context.get('positioning_client')
            
            if positioning_client is None:
                return {'status': 'error', 'error': 'Positioning client not initialized'}
            
            session_id = context.get('current_session_id')
            if session_id is None:
                return {'status': 'error', 'error': 'No active session'}
            
            template_points = operation.get('template_points', [])
            timeout = operation.get('timeout', 10000)
            
            print(f"    Computing 3D positions for session {session_id}")
            
            # Get positioning result
            result = positioning_client.get_result(
                session_id,
                template_points=template_points,
                timeout=timeout
            )
            
            if not result.get('success'):
                return {
                    'status': 'error',
                    'error': result.get('error', 'Positioning failed')
                }
            
            positioning_result = result.get('result', {})
            points_3d = positioning_result.get('points_3d', [])
            mean_error = positioning_result.get('mean_error', 0)
            
            print(f"    ✓ Positioning completed: {len(points_3d)} points, error={mean_error:.2f}px")
            
            return {
                'status': 'success',
                'outputs': {
                    'points_3d': points_3d,
                    'mean_error': mean_error,
                    'positioning_result': positioning_result
                }
            }
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
