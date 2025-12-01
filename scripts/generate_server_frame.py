#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import yaml
from robot_status_redis.client_utils import RobotStatusClient
from common.workspace_utils import get_workspace_root


class GenerateServerFrame:
    """Generate server frame by work object."""
    
    def __init__(self):

        # ======================= Configuration =======================
        # variables to configure rack position offsets
        self.rack_origin_to_unit_start_z = None
        self.rack_origin_to_unit_start_y = None

        # variables to store rack dimensions
        self.rack_inner_width = None
        self.rack_inner_height = None

        # variables to store information related to units
        self.unit_length = None

        # ======================= Instance Variables =======================
        self.robot_status_client = None
        
        # ======================= Variables =======================
        # variables to store rack coordinate system information (in base frame)
        self.rack_transformation_matrix_in_base = None
        self.rack_origin_in_base = None
        self.rack_x_axis_in_base = None
        self.rack_y_axis_in_base = None
        self.rack_z_axis_in_base = None
        
        # variables to store wobj coordinate system information (in rack frame)
        self.wobj_transformation_matrix_in_rack = None
        self.wobj_origin_in_rack = None
        self.wobj_x_axis_in_rack = None
        self.wobj_y_axis_in_rack = None
        self.wobj_z_axis_in_rack = None

        # ======================= Initialization =======================
        self._load_rack_config()
        self._initialize_robot_status_client()
        self._init_wobj_in_rack()
        self._load_rack2base_from_service()
    
    def _load_rack_config(self):
        """Load rack configuration from robot_config.yaml file.
        
        Returns:
            bool: True if successfully loaded, False otherwise
        """
        try:
            # Get the project root directory using common workspace utilities
            project_root = get_workspace_root()
            if project_root is None:
                print("Could not find workspace root directory")
                return False
            
            # Build config file path
            import os
            config_path = os.path.join(project_root, 'config', 'robot_config.yaml')
            
            # Check if config file exists
            if not os.path.exists(config_path):
                print(f"Config file not found: {config_path}")
                return False
            
            # Load YAML configuration
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # Extract GB200_rack configuration from 'shared' section
            if 'shared' not in config or 'GB200_rack' not in config['shared']:
                print("GB200_rack configuration not found in robot_config.yaml under 'shared' section")
                return False
            
            rack_config = config['shared']['GB200_rack']
            
            # Load rack parameters
            self.rack_origin_to_unit_start_z = rack_config.get('rack_origin_to_unit_start_z', 0.03715)
            self.rack_origin_to_unit_start_y = rack_config.get('rack_origin_to_unit_start_y', 0.190)
            self.rack_inner_width = rack_config.get('rack_inner_width', 0.550)
            self.rack_inner_height = rack_config.get('rack_inner_height', 2.141)
            self.unit_length = rack_config.get('unit_length', 0.04445)
            
            print("Successfully loaded rack configuration from robot_config.yaml")
            print(f"  rack_origin_to_unit_start_z: {self.rack_origin_to_unit_start_z}")
            print(f"  rack_origin_to_unit_start_y: {self.rack_origin_to_unit_start_y}")
            print(f"  rack_inner_width: {self.rack_inner_width}")
            print(f"  rack_inner_height: {self.rack_inner_height}")
            print(f"  unit_length: {self.unit_length}")
            return True
            
        except Exception as e:
            print(f"Error loading rack configuration: {e}")
            # Set default values if loading fails
            self.rack_origin_to_unit_start_z = 0.03715
            self.rack_origin_to_unit_start_y = 0.190
            self.rack_inner_width = 0.550
            self.rack_inner_height = 2.141
            self.unit_length = 0.04445
            print("Using default rack configuration values")
            return False
    
    def _initialize_robot_status_client(self):
        """Initialize robot status client."""
        try:
            self.robot_status_client = RobotStatusClient()
            print("Robot status client initialized successfully")
        except Exception as e:
            print(f"Error initializing robot status client: {e}")
            self.robot_status_client = None
    
    def _init_wobj_in_rack(self):
        """Initialize wobj coordinate system in wobj frame (identity transformation)."""
        self.wobj_origin_in_rack = np.array([0.0, 0.0, 0.0])
        self.wobj_x_axis_in_rack = np.array([1.0, 0.0, 0.0])
        self.wobj_y_axis_in_rack = np.array([0.0, 1.0, 0.0])
        self.wobj_z_axis_in_rack = np.array([0.0, 0.0, 1.0])
        self.wobj_transformation_matrix_in_rack = np.eye(4)
    
    def _load_rack2base_from_service(self):
        """Load rack to base transformation matrix from robot status service.
        
        Returns:
            bool: True if successfully loaded, False otherwise
        """
        if self.robot_status_client is None:
            print("Robot status client not initialized")
            return False
        
        try:
            # Get rack2base_matrix from ur15 namespace
            rack2base_matrix_data = self.robot_status_client.get_status('ur15', 'rack2base_matrix')
            
            if rack2base_matrix_data is None:
                print("rack2base_matrix not found in ur15 namespace")
                return False
            
            rack2base_matrix = np.array(rack2base_matrix_data)
            
            # Validate matrix shape
            if rack2base_matrix.shape != (4, 4):
                print(f"Invalid rack2base_matrix shape: {rack2base_matrix.shape}, expected (4, 4)")
                return False
            
            # Store the transformation matrix
            self.rack_transformation_matrix_in_base = rack2base_matrix
            
            # Extract origin and axes from the transformation matrix
            self.rack_origin_in_base = rack2base_matrix[:3, 3]
            self.rack_x_axis_in_base = rack2base_matrix[:3, 0]
            self.rack_y_axis_in_base = rack2base_matrix[:3, 1]
            self.rack_z_axis_in_base = rack2base_matrix[:3, 2]
            
            print("Successfully loaded rack2base transformation matrix")
            return True
            
        except Exception as e:
            print(f"Error loading rack2base_matrix from service: {e}")
            return False
    
    def generate_server_frame_in_rack(self, index=14):
        """Generate coordinate frame for target server in rack frame.
        
        Args:
            index (int): Server index, default is 14
            
        Returns:
            dict: Dictionary containing frame information in rack coordinate system
                  Returns None if rack information is not loaded
        """
        if self.wobj_transformation_matrix_in_rack is None:
            print("Rack coordinate system not initialized, cannot generate frame")
            return None
        
        try:
            # Calculate translation offsets in each direction
            offset_x = 0.5 * self.rack_inner_width
            offset_y = self.rack_origin_to_unit_start_y
            offset_z = self.rack_origin_to_unit_start_z + self.unit_length * (index-1)
            
            # Calculate new origin in rack frame (simple vector addition since rack axes are identity)
            target_server_origin_in_rack = np.array([offset_x, offset_y, offset_z])
            
            # Create transformation matrix for the new frame (in rack frame)
            # Pose (rotation) remains the same as rack (identity in rack frame)
            target_server_transformation_matrix_in_rack = np.eye(4)
            target_server_transformation_matrix_in_rack[:3, 3] = target_server_origin_in_rack  # New origin position
            
            # Extract frame axes (same as rack axes in rack frame, which are identity axes)
            target_server_x_axis_in_rack = self.wobj_x_axis_in_rack.copy()
            target_server_y_axis_in_rack = self.wobj_y_axis_in_rack.copy()
            target_server_z_axis_in_rack = self.wobj_z_axis_in_rack.copy()
            
            result = {
                'target_server_origin_in_rack': target_server_origin_in_rack,
                'target_server_x_axis_in_rack': target_server_x_axis_in_rack,
                'target_server_y_axis_in_rack': target_server_y_axis_in_rack,
                'target_server_z_axis_in_rack': target_server_z_axis_in_rack,
                'target_server_transformation_matrix_in_rack': target_server_transformation_matrix_in_rack
            }
            
            # Debug prints commented out to avoid spamming logs in video stream loop
            # print(f"Generated frame for server index {index} (in rack frame)")
            # print(f"Target server origin: {target_server_origin_in_rack}")
            # print(f"Target server X-axis: {target_server_x_axis_in_rack}")
            # print(f"Target server Y-axis: {target_server_y_axis_in_rack}")
            # print(f"Target server Z-axis: {target_server_z_axis_in_rack}")
            # print(f"Target server transformation matrix in rack:\n{target_server_transformation_matrix_in_rack}")
            
            return result
            
        except Exception as e:
            print(f"Error generating frame for target server in rack: {e}")
            return None
    
    def generate_server_frame_in_base(self, index=14):
        """Generate coordinate frame for target server in base frame.
        
        Args:
            index (int): Server index, default is 14
            
        Returns:
            dict: Dictionary containing frame information in base coordinate system
                  Returns None if rack information is not loaded
        """
        # Reload rack2base matrix from service to get latest updates
        if not self._load_rack2base_from_service():
            print("Failed to load rack2base matrix from service")
            return None
        
        try:
            # First generate the server frame in rack coordinate system
            result_in_rack = self.generate_server_frame_in_rack(index)
            
            if result_in_rack is None:
                print("Failed to generate server frame in rack")
                return None
            
            # Get the transformation matrix in rack frame
            target_server_transformation_matrix_in_rack = result_in_rack['target_server_transformation_matrix_in_rack']
            
            # Transform from rack frame to base frame
            # T_base_server = T_base_rack @ T_rack_server
            target_server_transformation_matrix_in_base = self.rack_transformation_matrix_in_base @ target_server_transformation_matrix_in_rack
            
            # Extract origin and axes from the transformation matrix in base frame
            target_server_origin_in_base = target_server_transformation_matrix_in_base[:3, 3]
            target_server_x_axis_in_base = target_server_transformation_matrix_in_base[:3, 0]
            target_server_y_axis_in_base = target_server_transformation_matrix_in_base[:3, 1]
            target_server_z_axis_in_base = target_server_transformation_matrix_in_base[:3, 2]
            
            result = {
                'target_server_origin_in_base': target_server_origin_in_base,
                'target_server_x_axis_in_base': target_server_x_axis_in_base,
                'target_server_y_axis_in_base': target_server_y_axis_in_base,
                'target_server_z_axis_in_base': target_server_z_axis_in_base,
                'target_server_transformation_matrix_in_base': target_server_transformation_matrix_in_base
            }
            
            # Debug prints commented out to avoid spamming logs in video stream loop
            # print(f"Generated frame for server index {index} (in base frame)")
            # print(f"Target server origin: {target_server_origin_in_base}")
            # print(f"Target server X-axis: {target_server_x_axis_in_base}")
            # print(f"Target server Y-axis: {target_server_y_axis_in_base}")
            # print(f"Target server Z-axis: {target_server_z_axis_in_base}")
            # print(f"Target server transformation matrix in base:\n{target_server_transformation_matrix_in_base}")
            
            return result
            
        except Exception as e:
            print(f"Error generating frame for target server in base: {e}")
            return None


def main():
    """Main function to generate and print server coordinate frame information."""
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Generate server frame for target server')
    parser.add_argument('--index', type=int, default=14, 
                       help='Server index (default: 14)')
    args = parser.parse_args()
        
    # Create instance
    generator = GenerateServerFrame()
    
    # Generate frame for target server in rack frame
    print("=" * 60)
    print(f"🎯 Generating frame for server index: {args.index}")
    print("=" * 60)

    print("\n" + "=" * 60)
    print("📍 Target Server Frame (wobj) in Rack Coordinate System:")
    print("=" * 60)
    result_rack = generator.generate_server_frame_in_rack(args.index)
    
    if result_rack:
        print("\n✅ Server frame in rack generated successfully!")
    else:
        print("\n❌ Failed to generate server frame in rack")
        return
    
    # Generate frame for target server in base frame
    print("\n" + "=" * 60)
    print("📍 Target Server Frame in Base Coordinate System:")
    print("=" * 60)
    result_base = generator.generate_server_frame_in_base(args.index)
    
    if result_base:
        print("\n✅ Server frame in base generated successfully!")
    else:
        print("\n❌ Failed to generate server frame in base")


if __name__ == "__main__":
    main()
