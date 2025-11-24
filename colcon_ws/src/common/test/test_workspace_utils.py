#!/usr/bin/env python3
"""
Tests for workspace_utils module.

These tests verify that workspace root and temp directory resolution
works correctly across different execution contexts.
"""

import os
import tempfile
import unittest
from unittest.mock import patch
import shutil

# Import the module under test
from common.workspace_utils import get_workspace_root, get_temp_directory


class TestGetWorkspaceRoot(unittest.TestCase):
    """Test cases for get_workspace_root() function."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Store original environment variables
        self.original_env = {
            'ROBOT_DC_ROOT': os.environ.get('ROBOT_DC_ROOT'),
            'COLCON_PREFIX_PATH': os.environ.get('COLCON_PREFIX_PATH'),
            'ROS_WORKSPACE': os.environ.get('ROS_WORKSPACE'),
        }
    
    def tearDown(self):
        """Restore original environment."""
        for key, value in self.original_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    
    def test_method0_robot_dc_root_env(self):
        """Test Method 0: ROBOT_DC_ROOT environment variable."""
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_dc_path = os.path.join(tmpdir, 'robot_dc')
            colcon_ws_path = os.path.join(robot_dc_path, 'colcon_ws')
            os.makedirs(colcon_ws_path)
            
            # Set environment variable
            os.environ['ROBOT_DC_ROOT'] = robot_dc_path
            
            # Clear other env vars that might interfere
            for key in ['COLCON_PREFIX_PATH', 'ROS_WORKSPACE']:
                os.environ.pop(key, None)
            
            result = get_workspace_root()
            self.assertEqual(result, robot_dc_path)
    
    def test_method1_ament_index(self):
        """Test Method 1: ament_index package discovery."""
        # This test verifies the logic but may not work in all environments
        # since it depends on actual ament_index installation
        
        # Clear environment variables that would take precedence
        for key in ['ROBOT_DC_ROOT', 'COLCON_PREFIX_PATH', 'ROS_WORKSPACE']:
            os.environ.pop(key, None)
        
        # When running from install space, this should work
        result = get_workspace_root()
        
        # Result should be a valid path
        self.assertIsNotNone(result)
        self.assertTrue(os.path.isabs(result))
        
        # Should end with robot_dc
        self.assertTrue(result.endswith('robot_dc'))
    
    def test_workspace_root_with_env_override(self):
        """Test that ROBOT_DC_ROOT overrides other detection methods."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a test robot_dc structure
            custom_root = os.path.join(tmpdir, 'custom_robot_dc')
            colcon_ws = os.path.join(custom_root, 'colcon_ws')
            os.makedirs(colcon_ws)
            
            # Set ROBOT_DC_ROOT to custom location
            os.environ['ROBOT_DC_ROOT'] = custom_root
            
            result = get_workspace_root()
            self.assertEqual(result, custom_root)
    
    def test_env_var_takes_precedence_over_detection(self):
        """Test that environment variable takes precedence over automatic detection."""
        # In a real workspace, env var should override automatic detection
        real_root = get_workspace_root()  # Get current root
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a different robot_dc
            override_root = os.path.join(tmpdir, 'override_robot_dc')
            colcon_ws = os.path.join(override_root, 'colcon_ws')
            os.makedirs(colcon_ws)
            
            # Override with environment variable
            os.environ['ROBOT_DC_ROOT'] = override_root
            
            try:
                result = get_workspace_root()
                self.assertEqual(result, override_root)
                self.assertNotEqual(result, real_root)
            finally:
                os.environ.pop('ROBOT_DC_ROOT', None)
    
    def test_invalid_robot_dc_root_ignored(self):
        """Test that invalid ROBOT_DC_ROOT is ignored."""
        # Set ROBOT_DC_ROOT to non-existent path
        os.environ['ROBOT_DC_ROOT'] = '/nonexistent/path/robot_dc'
        
        # Should fall back to other methods and find real workspace
        result = get_workspace_root()
        self.assertIsNotNone(result)
        self.assertTrue(os.path.exists(result))
        # Should NOT be the invalid path
        self.assertNotEqual(result, '/nonexistent/path/robot_dc')
    
    def test_priority_order(self):
        """Test that environment variables are checked in correct priority order."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create multiple potential roots
            robot_dc_root = os.path.join(tmpdir, 'robot_dc_from_env')
            colcon_ws_root = os.path.join(robot_dc_root, 'colcon_ws')
            ros_workspace = os.path.join(tmpdir, 'robot_dc_from_ros')
            
            os.makedirs(colcon_ws_root)
            os.makedirs(ros_workspace)
            
            # Set both environment variables
            os.environ['ROBOT_DC_ROOT'] = robot_dc_root
            os.environ['ROS_WORKSPACE'] = ros_workspace
            os.environ.pop('COLCON_PREFIX_PATH', None)
            
            # ROBOT_DC_ROOT should take precedence (Method 0)
            result = get_workspace_root()
            self.assertEqual(result, robot_dc_root)
    
    def test_returns_absolute_path(self):
        """Test that returned path is always absolute."""
        result = get_workspace_root()
        self.assertIsNotNone(result)
        self.assertTrue(os.path.isabs(result))
    
    def test_returns_existing_directory(self):
        """Test that returned path exists and is a directory."""
        result = get_workspace_root()
        self.assertIsNotNone(result)
        self.assertTrue(os.path.exists(result))
        self.assertTrue(os.path.isdir(result))


class TestGetTempDirectory(unittest.TestCase):
    """Test cases for get_temp_directory() function."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.original_env = {
            'ROBOT_DC_ROOT': os.environ.get('ROBOT_DC_ROOT'),
        }
        self.test_temp_dirs = []
    
    def tearDown(self):
        """Clean up test directories."""
        for key, value in self.original_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        
        # Clean up any test temp directories
        for temp_dir in self.test_temp_dirs:
            if os.path.exists(temp_dir):
                try:
                    shutil.rmtree(temp_dir)
                except Exception:
                    pass
    
    def test_creates_temp_directory(self):
        """Test that temp directory is created if it doesn't exist."""
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_dc = os.path.join(tmpdir, 'robot_dc')
            colcon_ws = os.path.join(robot_dc, 'colcon_ws')
            os.makedirs(colcon_ws)
            
            os.environ['ROBOT_DC_ROOT'] = robot_dc
            
            temp_dir = get_temp_directory()
            self.test_temp_dirs.append(temp_dir)
            
            # Verify temp directory exists
            self.assertTrue(os.path.exists(temp_dir))
            self.assertTrue(os.path.isdir(temp_dir))
            
            # Verify it's in the right location
            expected_path = os.path.join(robot_dc, 'temp')
            self.assertEqual(temp_dir, expected_path)
    
    def test_returns_existing_temp_directory(self):
        """Test that existing temp directory is returned."""
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_dc = os.path.join(tmpdir, 'robot_dc')
            colcon_ws = os.path.join(robot_dc, 'colcon_ws')
            temp_dir = os.path.join(robot_dc, 'temp')
            os.makedirs(colcon_ws)
            os.makedirs(temp_dir)
            
            os.environ['ROBOT_DC_ROOT'] = robot_dc
            
            result = get_temp_directory()
            self.assertEqual(result, temp_dir)
            self.assertTrue(os.path.exists(result))
    
    def test_raises_error_when_workspace_not_found(self):
        """Test that RuntimeError is raised when workspace root cannot be found."""
        # Clear all env vars and mock everything to fail
        for key in ['ROBOT_DC_ROOT', 'COLCON_PREFIX_PATH', 'ROS_WORKSPACE']:
            os.environ.pop(key, None)
        
        # Mock all detection methods to fail
        with patch('ament_index_python.packages.get_package_share_directory', side_effect=Exception):
            with patch('os.getcwd', return_value='/tmp'):
                with patch('os.path.exists', return_value=False):
                    with self.assertRaises(RuntimeError) as context:
                        get_temp_directory()
                    
                    self.assertIn('project root', str(context.exception).lower())
    
    def test_temp_directory_is_writable(self):
        """Test that returned temp directory is writable."""
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_dc = os.path.join(tmpdir, 'robot_dc')
            colcon_ws = os.path.join(robot_dc, 'colcon_ws')
            os.makedirs(colcon_ws)
            
            os.environ['ROBOT_DC_ROOT'] = robot_dc
            
            temp_dir = get_temp_directory()
            self.test_temp_dirs.append(temp_dir)
            
            # Try to create a test file
            test_file = os.path.join(temp_dir, 'test_file.txt')
            with open(test_file, 'w') as f:
                f.write('test')
            
            self.assertTrue(os.path.exists(test_file))
            
            # Clean up
            os.remove(test_file)
    
    def test_returns_absolute_path(self):
        """Test that returned path is always absolute."""
        with tempfile.TemporaryDirectory() as tmpdir:
            robot_dc = os.path.join(tmpdir, 'robot_dc')
            colcon_ws = os.path.join(robot_dc, 'colcon_ws')
            os.makedirs(colcon_ws)
            
            os.environ['ROBOT_DC_ROOT'] = robot_dc
            
            result = get_temp_directory()
            self.test_temp_dirs.append(result)
            
            self.assertTrue(os.path.isabs(result))


class TestIntegration(unittest.TestCase):
    """Integration tests for workspace utilities."""
    
    def test_real_workspace_detection(self):
        """Test workspace detection in real environment."""
        # This test uses the actual workspace
        workspace_root = get_workspace_root()
        
        # Should successfully find workspace
        self.assertIsNotNone(workspace_root)
        self.assertTrue(os.path.exists(workspace_root))
        
        # Should end with robot_dc
        self.assertTrue(workspace_root.endswith('robot_dc'))
        
        # Should have expected subdirectories
        colcon_ws_path = os.path.join(workspace_root, 'colcon_ws')
        scripts_path = os.path.join(workspace_root, 'scripts')
        
        self.assertTrue(os.path.exists(colcon_ws_path))
        self.assertTrue(os.path.exists(scripts_path))
    
    def test_temp_directory_in_real_workspace(self):
        """Test temp directory creation in real workspace."""
        temp_dir = get_temp_directory()
        
        # Should successfully get temp directory
        self.assertIsNotNone(temp_dir)
        self.assertTrue(os.path.exists(temp_dir))
        self.assertTrue(os.path.isdir(temp_dir))
        
        # Should be at robot_dc/temp
        self.assertTrue(temp_dir.endswith(os.path.join('robot_dc', 'temp')))
        
        # Should be writable
        test_file = os.path.join(temp_dir, 'test_workspace_utils.txt')
        try:
            with open(test_file, 'w') as f:
                f.write('test')
            self.assertTrue(os.path.exists(test_file))
        finally:
            if os.path.exists(test_file):
                os.remove(test_file)


if __name__ == '__main__':
    unittest.main()
