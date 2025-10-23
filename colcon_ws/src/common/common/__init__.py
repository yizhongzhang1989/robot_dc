"""
Common utilities and shared functionality for the robot_dc workspace.
"""

__version__ = '1.0.0'

# Import commonly used utilities for easy access
from .workspace_utils import (
    get_workspace_root,
    get_temp_directory,
    get_scripts_directory,
    get_calibration_images_directory
)

# Export public API
__all__ = [
    'get_workspace_root',
    'get_temp_directory', 
    'get_scripts_directory',
    'get_calibration_images_directory'
]
