"""
Workspace utility functions for ROS 2 projects.

This module provides utilities for finding and working with ROS 2 workspace
directories, regardless of where the package is installed or executed from.
"""

import os
from typing import Optional


def get_workspace_root() -> Optional[str]:
    """
    Find the ROS workspace root directory using multiple methods.
    
    This function attempts to locate the workspace root using several strategies:
    1. ROS package discovery (for installed packages)
    2. Filesystem structure search (for development)
    3. Environment variables (for various ROS setups)
    4. Fallback path (for compatibility)
    
    Returns:
        str: Path to the workspace root, or None if not found.
        
    Example:
        >>> workspace_root = get_workspace_root()
        >>> if workspace_root:
        ...     temp_dir = os.path.join(workspace_root, 'temp')
        >>> else:
        ...     raise RuntimeError("Could not find workspace root")
    """
    
    # Method 1: Use ROS package discovery
    # This works when the package is properly installed
    try:
        from ament_index_python.packages import get_package_share_directory
        
        # Try to get any known package in our workspace
        for package_name in ['common', 'robot_arm_web', 'camera_node']:
            try:
                package_dir = get_package_share_directory(package_name)
                current = package_dir
                while current != '/' and current:
                    parent = os.path.dirname(current)
                    if os.path.basename(current) in ['install', 'share']:
                        # Check if parent has src, install, build directories
                        potential_ws = parent if os.path.basename(current) == 'install' else parent
                        if (os.path.exists(os.path.join(potential_ws, 'src')) and 
                            os.path.exists(os.path.join(potential_ws, 'install'))):
                            return potential_ws
                    current = parent
            except Exception:
                continue
    except Exception:
        pass
    
    # Method 2: Search from current file location
    # This works during development when running from source
    current = os.path.dirname(os.path.abspath(__file__))
    while current != '/' and current:
        # Look for ROS workspace structure
        if (os.path.exists(os.path.join(current, 'src')) and 
            (os.path.exists(os.path.join(current, 'install')) or 
             os.path.exists(os.path.join(current, 'build')))):
            return current
        # Check if we're in a colcon_ws directory
        if os.path.basename(current) == 'colcon_ws':
            return os.path.dirname(current)
        current = os.path.dirname(current)
    
    # Method 3: Check environment variables
    # This works in various ROS setups where environment is properly sourced
    if 'COLCON_PREFIX_PATH' in os.environ:
        install_path = os.environ['COLCON_PREFIX_PATH'].split(':')[0]
        potential_ws = os.path.dirname(install_path)
        if os.path.exists(os.path.join(potential_ws, 'src')):
            return potential_ws
    
    # Method 4: Check ROS workspace environment variable if set
    if 'ROS_WORKSPACE' in os.environ:
        workspace_path = os.environ['ROS_WORKSPACE']
        if os.path.exists(workspace_path) and os.path.exists(os.path.join(workspace_path, 'src')):
            return workspace_path
    
    # Method 5: Fallback to common development paths
    fallback_paths = [
        '/home/a/Documents/robot_dc',
        '/home/a/Documents/robot_dc2',  # Legacy path
        os.path.expanduser('~/robot_dc'),
        os.path.expanduser('~/Documents/robot_dc'),
    ]
    
    for fallback_path in fallback_paths:
        if os.path.exists(fallback_path) and os.path.exists(os.path.join(fallback_path, 'colcon_ws')):
            return fallback_path
    
    return None


def get_temp_directory() -> str:
    """
    Get the temp directory path within the workspace.
    
    This function finds the workspace root and creates/returns the temp directory path.
    The temp directory is automatically created if it doesn't exist.
    
    Returns:
        str: Path to the temp directory.
        
    Raises:
        RuntimeError: If workspace root cannot be found.
        
    Example:
        >>> try:
        ...     temp_dir = get_temp_directory()
        ...     calibration_dir = temp_dir  # Use for calibration files
        ... except RuntimeError as e:
        ...     print(f"Error: {e}")
    """
    workspace_root = get_workspace_root()
    if workspace_root is None:
        raise RuntimeError(
            "Could not determine workspace root directory. "
            "Make sure you're running from within a ROS workspace or set the ROS_WORKSPACE environment variable."
        )
    
    temp_dir = os.path.join(workspace_root, 'temp')
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir


def get_scripts_directory() -> Optional[str]:
    """
    Get the scripts directory path within the workspace.
    
    Returns:
        str: Path to the scripts directory, or None if not found.
        
    Example:
        >>> scripts_dir = get_scripts_directory()
        >>> if scripts_dir:
        ...     calibration_toolkit = os.path.join(scripts_dir, 'ThirdParty', 'camera_calibration_toolkit')
    """
    workspace_root = get_workspace_root()
    if workspace_root is None:
        return None
    
    # Check in workspace root first
    scripts_dir = os.path.join(workspace_root, 'scripts')
    if os.path.exists(scripts_dir):
        return scripts_dir
    
    # Check in parent directory (for colcon_ws structure)
    parent_dir = os.path.dirname(workspace_root)
    parent_scripts_dir = os.path.join(parent_dir, 'scripts')
    if os.path.exists(parent_scripts_dir):
        return parent_scripts_dir
    
    return None


def get_calibration_images_directory() -> str:
    """
    Get the calibration images directory path within the workspace.
    
    This is a convenience function that returns the path where calibration
    images should be stored, typically in the workspace's calibration_images directory.
    
    Returns:
        str: Path to the calibration images directory.
        
    Raises:
        RuntimeError: If workspace root cannot be found.
    """
    workspace_root = get_workspace_root()
    if workspace_root is None:
        raise RuntimeError("Could not determine workspace root directory")
    
    calibration_dir = os.path.join(workspace_root, 'calibration_images')
    os.makedirs(calibration_dir, exist_ok=True)
    return calibration_dir
