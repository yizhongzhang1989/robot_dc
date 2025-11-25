#!/usr/bin/env python3
"""
Tests for ConfigManager

Run tests with:
    cd colcon_ws
    colcon test --packages-select common --pytest-args test/test_config_manager.py
    
Or directly:
    cd colcon_ws/src/common
    pytest test/test_config_manager.py -v
"""

import os
import pytest
import tempfile
import yaml
from pathlib import Path
from unittest.mock import patch, MagicMock

from common.config_manager import ConfigManager, RobotConfig, ConfigError, get_config


@pytest.fixture
def temp_config_dir():
    """Create a temporary config directory."""
    with tempfile.TemporaryDirectory() as tmpdir:
        config_dir = Path(tmpdir) / 'config'
        config_dir.mkdir()
        yield config_dir


@pytest.fixture
def sample_config():
    """Sample configuration for testing."""
    return {
        'version': '1.0',
        'ur15': {
            'network': {
                'robot_ip': '192.168.1.15',
                'ports': {
                    'control': 30002,
                    'dashboard': 29999
                },
                'camera': {
                    'name': 'UR15Camera',
                    'ip': '192.168.1.101',
                    'rtsp_url': 'rtsp://admin:password@192.168.1.101/stream0'
                }
            },
            'services': {
                'web': 8030,
                'positioning_3d': 8004
            },
            'paths': {
                'dataset': 'dataset',
                'calibration_data': 'temp/ur15_calibration'
            }
        },
        'duco': {
            'network': {
                'robot_ip': '192.168.1.10',
                'port': 7003
            },
            'services': {
                'web': 8031
            }
        },
        'shared': {
            'network': {
                'redis': {
                    'host': 'localhost',
                    'port': 6379,
                    'db': 0
                }
            }
        }
    }


@pytest.fixture
def config_file(temp_config_dir, sample_config):
    """Create a temporary config file."""
    config_path = temp_config_dir / 'robot_config.yaml'
    with open(config_path, 'w') as f:
        yaml.dump(sample_config, f)
    return config_path


@pytest.fixture(autouse=True)
def reset_singleton():
    """Reset ConfigManager singleton between tests."""
    ConfigManager._instance = None
    ConfigManager._initialized = False
    yield
    ConfigManager._instance = None
    ConfigManager._initialized = False


class TestConfigManager:
    """Test ConfigManager functionality."""
    
    def test_singleton_pattern(self, config_file):
        """Test that ConfigManager is a singleton."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config1 = ConfigManager()
            config2 = ConfigManager()
            assert config1 is config2
    
    def test_load_config_success(self, config_file, sample_config):
        """Test successful configuration loading."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            assert config.get('version') == '1.0'
            assert config.get('ur15.network.robot_ip') == '192.168.1.15'
    
    def test_config_not_found(self):
        """Test error when config file not found."""
        with patch.object(ConfigManager, '_find_config_file', return_value=None):
            with pytest.raises(ConfigError, match="Configuration file not found"):
                ConfigManager()
    
    def test_get_with_dot_notation(self, config_file):
        """Test get() with dot notation."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            # Top level
            assert config.get('version') == '1.0'
            
            # Nested
            assert config.get('ur15.network.robot_ip') == '192.168.1.15'
            assert config.get('ur15.network.ports.control') == 30002
            
            # Deep nested
            assert config.get('ur15.network.camera.name') == 'UR15Camera'
    
    def test_get_with_default(self, config_file):
        """Test get() with default value."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            # Existing key
            assert config.get('ur15.services.web', default=9999) == 8030
            
            # Non-existing key
            assert config.get('nonexistent.key', default='default_value') == 'default_value'
            assert config.get('ur15.nonexistent', default=None) is None
    
    def test_has_key(self, config_file):
        """Test has() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            assert config.has('ur15.network.robot_ip') is True
            assert config.has('ur15.services.web') is True
            assert config.has('nonexistent.key') is False
            assert config.has('ur15.nonexistent') is False
    
    def test_dict_access(self, config_file):
        """Test dictionary-style access."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            assert config['ur15']['network']['robot_ip'] == '192.168.1.15'
            assert config['shared']['network']['redis']['host'] == 'localhost'
    
    def test_get_robot(self, config_file):
        """Test get_robot() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            # Get UR15
            ur15 = config.get_robot('ur15')
            assert isinstance(ur15, RobotConfig)
            assert ur15.name == 'ur15'
            assert ur15.get('network.robot_ip') == '192.168.1.15'
            
            # Get Duco
            duco = config.get_robot('duco')
            assert duco.name == 'duco'
            assert duco.get('network.robot_ip') == '192.168.1.10'
    
    def test_get_robot_not_found(self, config_file):
        """Test get_robot() with non-existent robot."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            with pytest.raises(ConfigError, match="Robot 'nonexistent' not found"):
                config.get_robot('nonexistent')
    
    def test_list_robots(self, config_file):
        """Test list_robots() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            robots = config.list_robots()
            assert 'ur15' in robots
            assert 'duco' in robots
            assert 'shared' not in robots
            assert 'version' not in robots
    
    def test_get_all(self, config_file, sample_config):
        """Test get_all() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            
            all_config = config.get_all()
            assert all_config['version'] == sample_config['version']
            # Paths are resolved to absolute, so just check keys exist
            assert 'ur15' in all_config
            assert 'network' in all_config['ur15']
            assert 'services' in all_config['ur15']


class TestRobotConfig:
    """Test RobotConfig wrapper functionality."""
    
    def test_robot_config_get(self, config_file):
        """Test RobotConfig get() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            ur15 = config.get_robot('ur15')
            
            assert ur15.get('network.robot_ip') == '192.168.1.15'
            assert ur15.get('services.web') == 8030
            assert ur15.get('network.ports.control') == 30002
    
    def test_robot_config_get_default(self, config_file):
        """Test RobotConfig get() with default."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            ur15 = config.get_robot('ur15')
            
            assert ur15.get('nonexistent', default='default') == 'default'
    
    def test_robot_config_has(self, config_file):
        """Test RobotConfig has() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            ur15 = config.get_robot('ur15')
            
            assert ur15.has('network.robot_ip') is True
            assert ur15.has('nonexistent') is False
    
    def test_robot_config_dict_access(self, config_file):
        """Test RobotConfig dictionary-style access."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            ur15 = config.get_robot('ur15')
            
            assert ur15['network']['robot_ip'] == '192.168.1.15'
    
    def test_robot_config_get_all(self, config_file):
        """Test RobotConfig get_all() method."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = ConfigManager()
            ur15 = config.get_robot('ur15')
            
            all_data = ur15.get_all()
            assert 'network' in all_data
            assert 'services' in all_data


class TestEnvironmentVariables:
    """Test environment variable expansion."""
    
    def test_env_var_expansion(self, temp_config_dir):
        """Test ${VAR_NAME} expansion in config."""
        config_data = {
            'version': '1.0',
            'ur15': {
                'network': {
                    'robot_ip': '${ROBOT_IP}',
                    'camera': {
                        'rtsp_url': 'rtsp://admin:${CAMERA_PASSWORD}@192.168.1.101/stream0'
                    }
                }
            }
        }
        
        config_path = temp_config_dir / 'robot_config.yaml'
        with open(config_path, 'w') as f:
            yaml.dump(config_data, f)
        
        # Set environment variables
        os.environ['ROBOT_IP'] = '192.168.1.99'
        os.environ['CAMERA_PASSWORD'] = 'secret123'
        
        try:
            with patch.object(ConfigManager, '_find_config_file', return_value=config_path):
                config = ConfigManager()
                
                assert config.get('ur15.network.robot_ip') == '192.168.1.99'
                assert 'secret123' in config.get('ur15.network.camera.rtsp_url')
        finally:
            del os.environ['ROBOT_IP']
            del os.environ['CAMERA_PASSWORD']
    
    def test_env_var_not_set(self, temp_config_dir):
        """Test behavior when env var not set."""
        config_data = {
            'version': '1.0',
            'ur15': {
                'network': {
                    'robot_ip': '${UNDEFINED_VAR}'
                }
            }
        }
        
        config_path = temp_config_dir / 'robot_config.yaml'
        with open(config_path, 'w') as f:
            yaml.dump(config_data, f)
        
        with patch.object(ConfigManager, '_find_config_file', return_value=config_path):
            config = ConfigManager()
            # Undefined variables are left as-is
            assert config.get('ur15.network.robot_ip') == '${UNDEFINED_VAR}'


class TestPathResolution:
    """Test path resolution functionality."""
    
    def test_path_resolution(self, temp_config_dir):
        """Test that relative paths are resolved."""
        config_data = {
            'version': '1.0',
            'ur15': {
                'paths': {
                    'dataset': 'dataset',
                    'calibration': 'temp/calibration'
                }
            }
        }
        
        config_path = temp_config_dir / 'robot_config.yaml'
        with open(config_path, 'w') as f:
            yaml.dump(config_data, f)
        
        workspace_root = temp_config_dir.parent
        
        # Patch get_workspace_root in workspace_utils module
        with patch('common.workspace_utils.get_workspace_root', return_value=str(workspace_root)):
            with patch.object(ConfigManager, '_find_config_file', return_value=config_path):
                config = ConfigManager()
                
                dataset_path = config.get('ur15.paths.dataset')
                calib_path = config.get('ur15.paths.calibration')
                
                # Paths should be absolute
                assert Path(dataset_path).is_absolute()
                assert Path(calib_path).is_absolute()
                assert dataset_path.endswith('dataset')
                assert calib_path.endswith('calibration')


class TestConvenienceFunctions:
    """Test convenience functions."""
    
    def test_get_config(self, config_file):
        """Test get_config() convenience function."""
        with patch.object(ConfigManager, '_find_config_file', return_value=config_file):
            config = get_config()
            assert isinstance(config, ConfigManager)
            assert config.get('version') == '1.0'


class TestReload:
    """Test configuration reload."""
    
    def test_reload(self, temp_config_dir):
        """Test reload() method."""
        config_path = temp_config_dir / 'robot_config.yaml'
        
        # Initial config
        config_data1 = {
            'version': '1.0',
            'ur15': {'network': {'robot_ip': '192.168.1.15'}}
        }
        with open(config_path, 'w') as f:
            yaml.dump(config_data1, f)
        
        with patch.object(ConfigManager, '_find_config_file', return_value=config_path):
            config = ConfigManager()
            assert config.get('ur15.network.robot_ip') == '192.168.1.15'
            
            # Update config file
            config_data2 = {
                'version': '1.0',
                'ur15': {'network': {'robot_ip': '192.168.1.99'}}
            }
            with open(config_path, 'w') as f:
                yaml.dump(config_data2, f)
            
            # Reload
            config.reload()
            assert config.get('ur15.network.robot_ip') == '192.168.1.99'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
