#!/usr/bin/env python3
"""
Movement Pattern Handler
"""

import sys
import time
from pathlib import Path
from typing import Dict, Any, List
from common.workspace_utils import get_scripts_directory

scripts_dir = get_scripts_directory()
if scripts_dir:
    sys.path.append(scripts_dir)

from ur15_workflow.base import OperationHandler


class MovementPatternHandler(OperationHandler):
    """
    Handler for executing movement patterns (e.g., grid scan)
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute movement pattern with capture at each position
        
        Operation parameters:
            - pattern_type: 'grid_5point', 'grid_9point', 'circular', etc.
            - center_pose: Center position for pattern
            - pattern_params: Parameters for pattern (e.g., offset distance)
            - actions_at_each_position: List of actions to perform at each position
        """
        try:
            pattern_type = operation.get('pattern_type', 'grid_5point')
            pattern_params = operation.get('pattern_params', {})
            
            # Generate movement offsets based on pattern type
            offsets = self._generate_pattern_offsets(pattern_type, pattern_params)
            
            print(f"    Executing {pattern_type} pattern with {len(offsets)} positions")
            
            # Get robot from context
            robot = context.get('robot')
            if robot is None:
                return {'status': 'error', 'error': 'Robot not initialized in context'}
            
            # Execute pattern
            captured_data = []
            
            for idx, offset in enumerate(offsets):
                print(f"      Position {idx+1}/{len(offsets)}: offset={offset}")
                
                # Apply offset (TCP movement)
                if any(offset):
                    result_code = robot.move_tcp(offset, a=0.1, v=0.1)
                    if result_code != 0:
                        print("        ✗ Failed to apply offset")
                        continue
                    time.sleep(1.0)
                
                # Perform actions at this position (e.g., capture, save)
                # This would be expanded based on actions_at_each_position parameter
                
                # Return to center
                if any(offset):
                    reverse_offset = [-x for x in offset]
                    robot.move_tcp(reverse_offset, a=0.1, v=0.05)
                    time.sleep(0.5)
                
                captured_data.append({
                    'position_index': idx,
                    'offset': offset
                })
            
            return {
                'status': 'success',
                'outputs': {
                    'pattern_positions': len(offsets),
                    'captured_data': captured_data
                }
            }
            
        except Exception as e:
            return {'status': 'error', 'error': str(e)}
    
    def _generate_pattern_offsets(self, pattern_type: str, params: Dict) -> List[List[float]]:
        """
        Generate movement offsets based on pattern type
        
        Args:
            pattern_type: Type of pattern
            params: Pattern parameters
            
        Returns:
            List of [x, y, z, rx, ry, rz] offsets
        """
        offset_distance = params.get('offset_distance', 0.01)  # 1cm default
        
        if pattern_type == 'grid_5point':
            return [
                [0, 0, 0, 0, 0, 0],                          # Center
                [offset_distance, 0, 0, 0, 0, 0],            # +X
                [-offset_distance, 0, 0, 0, 0, 0],           # -X
                [0, offset_distance, 0, 0, 0, 0],            # +Y
                [0, -offset_distance, 0, 0, 0, 0]            # -Y
            ]
        
        elif pattern_type == 'grid_9point':
            return [
                [0, 0, 0, 0, 0, 0],
                [offset_distance, 0, 0, 0, 0, 0],
                [-offset_distance, 0, 0, 0, 0, 0],
                [0, offset_distance, 0, 0, 0, 0],
                [0, -offset_distance, 0, 0, 0, 0],
                [offset_distance, offset_distance, 0, 0, 0, 0],
                [offset_distance, -offset_distance, 0, 0, 0, 0],
                [-offset_distance, offset_distance, 0, 0, 0, 0],
                [-offset_distance, -offset_distance, 0, 0, 0, 0]
            ]
        
        elif pattern_type == 'single_point':
            return [[0, 0, 0, 0, 0, 0]]
        
        else:
            # Default to single point
            return [[0, 0, 0, 0, 0, 0]]
