#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from robot_status_redis.client_utils import RobotStatusClient


class GenerateServerFrame:
    """Generate server frame by work object."""
    
    def __init__(self):

        # ======================= Configuration =======================
        # variables to configure rack position offsets
        self.rack_origin_to_unit_start_z = 0.03815
        self.rack_origin_to_unit_start_y = 0.190

        # variables to store rack dimensions
        self.rack_inner_width = 0.550
        self.rack_inner_height = 2.145

        # variables to store information related to units
        self.unit_length= 0.04445

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
        self._initialize_robot_status_client()
        self._init_wobj_in_rack()
        self._load_rack2base_from_service()
    
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
