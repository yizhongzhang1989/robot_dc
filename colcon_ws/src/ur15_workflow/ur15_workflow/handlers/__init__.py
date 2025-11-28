"""
Operation Handlers Package

Contains concrete implementations of operation handlers for the workflow engine.
"""

from .robot_move import RobotMoveHandler
from .capture import CaptureImageHandler
from .movement_pattern import MovementPatternHandler
from .positioning import PositioningHandler
from .coordinate_frame import CoordinateFrameHandler

__all__ = [
    'RobotMoveHandler',
    'CaptureImageHandler',
    'MovementPatternHandler',
    'PositioningHandler',
    'CoordinateFrameHandler',
]


def register_all_handlers(workflow_engine):
    """
    Register all built-in handlers with the workflow engine
    
    Args:
        workflow_engine: WorkflowEngine instance
    """
    workflow_engine.register_handler('robot_move', RobotMoveHandler)
    workflow_engine.register_handler('capture_image', CaptureImageHandler)
    workflow_engine.register_handler('movement_pattern', MovementPatternHandler)
    workflow_engine.register_handler('positioning', PositioningHandler)
    workflow_engine.register_handler('coordinate_frame', CoordinateFrameHandler)
    
    print("✓ All operation handlers registered")
