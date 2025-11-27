#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from robot_status_redis.client_utils import RobotStatusClient


class GenerateServerFrame:
    """Generate server frame by work object."""
    
    def __init__(self):

        # ======================= Configuration =======================
        # variables to configure rack position offsets
        self.origin_to_unit_start_z = 0.015
        self.origin_to_unit_start_y = 0.19

        # variables to store rack dimensions
        self.rack_inner_width = 0.550
        self.rack_inner_height = 2.145

        # variables to store information related to units
        self.unit_length= 0.044
        self.interval_between_units = 0.0005

        self.offset_from_floor_bottom_to_server_top = 0.019
        self.offset_from_floor_bottom_to_server_bottom = 0.025

        # ======================= Instance Variables =======================
        self.robot_status_client = None
        
        # ======================= Variables =======================
        # variables to store wobj coordinate system information (in base frame)
        self.wobj_transformation_matrix_in_base = None
        self.wobj_origin_in_base = None
        self.wobj_x_in_base = None
        self.wobj_y_in_base = None
        self.wobj_z_in_base = None
        
        # variables to store wobj coordinate system information (in wobj frame)
        self.wobj_transformation_matrix_in_wobj = None
        self.wobj_origin_in_wobj = None
        self.wobj_x_in_wobj = None
        self.wobj_y_in_wobj = None
        self.wobj_z_in_wobj = None

        # ======================= Initialization =======================
        self._init_wobj_in_wobj()
        self._initialize_robot_status_client()
        self._load_wobj_from_service()
    
    def _initialize_robot_status_client(self):
        """Initialize robot status client."""
        try:
            self.robot_status_client = RobotStatusClient()
            print("Robot status client initialized successfully")
        except Exception as e:
            print(f"Error initializing robot status client: {e}")
            self.robot_status_client = None
    
    def _load_wobj_from_service(self):
        """Load work object coordinate system information from robot_status service.
        
        Returns:
            bool: True if successful, None otherwise
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping wobj information loading")
            return None
        
        try:
            # Get wobj_origin
            wobj_origin = self.robot_status_client.get_status('wobj', 'wobj_origin')
            if wobj_origin is None:
                print(f"Failed to get wobj_origin from robot status for 'wobj'")
                return None
            
            # Get wobj_x axis
            wobj_x = self.robot_status_client.get_status('wobj', 'wobj_x')
            if wobj_x is None:
                print(f"Failed to get wobj_x from robot status for 'wobj'")
                return None
            
            # Get wobj_y axis
            wobj_y = self.robot_status_client.get_status('wobj', 'wobj_y')
            if wobj_y is None:
                print(f"Failed to get wobj_y from robot status for 'wobj'")
                return None
            
            # Get wobj_z axis
            wobj_z = self.robot_status_client.get_status('wobj', 'wobj_z')
            if wobj_z is None:
                print(f"Failed to get wobj_z from robot status for 'wobj'")
                return None
            
            # Get transformation_matrix directly from service
            transformation_matrix_list = self.robot_status_client.get_status('wobj', 'transformation_matrix')
            if transformation_matrix_list is None:
                print(f"Failed to get transformation_matrix from robot status for 'wobj'")
                return None
            
            # Convert to numpy arrays and store as self variables (in base frame)
            self.wobj_origin_in_base = np.array(wobj_origin)
            self.wobj_x_in_base = np.array(wobj_x)
            self.wobj_y_in_base = np.array(wobj_y)
            self.wobj_z_in_base = np.array(wobj_z)
            self.wobj_transformation_matrix_in_base = np.array(transformation_matrix_list)
            
            print(f"Wobj coordinate system loaded successfully from robot status")            
            return True
            
        except Exception as e:
            print(f"Error loading wobj coordinate system: {e}")
            return None
    
    def _init_wobj_in_wobj(self):
        """Initialize wobj coordinate system in wobj frame (identity transformation)."""
        self.wobj_origin_in_wobj = np.array([0.0, 0.0, 0.0])
        self.wobj_x_in_wobj = np.array([1.0, 0.0, 0.0])
        self.wobj_y_in_wobj = np.array([0.0, 1.0, 0.0])
        self.wobj_z_in_wobj = np.array([0.0, 0.0, 1.0])
        self.wobj_transformation_matrix_in_wobj = np.eye(4)
    
    def generate_target_floor_frame_in_base(self, index=14):
        """Generate coordinate frame for target floor.
        
        Args:
            index (int): Floor index, default is 14
            
        Returns:
            dict: Dictionary containing 'origin', 'pose', and 'transformation_matrix'
                  Returns None if wobj information is not loaded
        """
        if self.wobj_transformation_matrix_in_base is None or self.wobj_origin_in_base is None:
            print("Wobj coordinate system not loaded, cannot generate frame")
            return None
        
        try:
            # Calculate translation offsets in each direction
            offset_x = 0.5 * self.rack_inner_width
            offset_y = self.origin_to_unit_start_y
            offset_z = self.origin_to_unit_start_z + self.unit_length * index + (index - 1) * self.interval_between_units
            
            # Calculate new origin by translating along wobj axes (in base frame)
            target_floor_origin_in_base = (self.wobj_origin_in_base + 
                                           offset_x * self.wobj_x_in_base + 
                                           offset_y * self.wobj_y_in_base + 
                                           offset_z * self.wobj_z_in_base)
            
            # Create transformation matrix for the new frame (in base frame)
            # Pose (rotation) remains the same as wobj
            target_floor_transformation_matrix_in_base = np.eye(4)
            target_floor_transformation_matrix_in_base[:3, :3] = self.wobj_transformation_matrix_in_base[:3, :3]  # Same rotation as wobj
            target_floor_transformation_matrix_in_base[:3, 3] = target_floor_origin_in_base  # New origin position
            
            # Extract frame axes from transformation matrix (same as wobj axes, in base frame)
            target_floor_x_in_base = self.wobj_x_in_base.copy()
            target_floor_y_in_base = self.wobj_y_in_base.copy()
            target_floor_z_in_base = self.wobj_z_in_base.copy()
            
            result = {
                'target_floor_origin_in_base': target_floor_origin_in_base,
                'target_floor_x_in_base': target_floor_x_in_base,
                'target_floor_y_in_base': target_floor_y_in_base,
                'target_floor_z_in_base': target_floor_z_in_base,
                'target_floor_transformation_matrix_in_base': target_floor_transformation_matrix_in_base
            }
            
            print(f"Generated frame for floor index {index} (in base frame)")
            print(f"Target floor origin: {target_floor_origin_in_base}")
            print(f"Target floor X-axis: {target_floor_x_in_base}")
            print(f"Target floor Y-axis: {target_floor_y_in_base}")
            print(f"Target floor Z-axis: {target_floor_z_in_base}")
            print(f"Target floor transformation matrix in base:\n{target_floor_transformation_matrix_in_base}")
            
            return result
            
        except Exception as e:
            print(f"Error generating frame for target floor: {e}")
            return None
    
    def generate_target_floor_frame_in_wobj(self, index=14):
        """Generate coordinate frame for target floor in wobj frame.
        
        Args:
            index (int): Floor index, default is 14
            
        Returns:
            dict: Dictionary containing frame information in wobj coordinate system
                  Returns None if wobj information is not loaded
        """
        if self.wobj_transformation_matrix_in_wobj is None:
            print("Wobj coordinate system not initialized, cannot generate frame")
            return None
        
        try:
            # Calculate translation offsets in each direction
            offset_x = 0.5 * self.rack_inner_width
            offset_y = self.origin_to_unit_start_y
            offset_z = self.origin_to_unit_start_z + self.unit_length * index + (index - 1) * self.interval_between_units
            
            # Calculate new origin in wobj frame (simple vector addition since wobj axes are identity)
            target_floor_origin_in_wobj = np.array([offset_x, offset_y, offset_z])
            
            # Create transformation matrix for the new frame (in wobj frame)
            # Pose (rotation) remains the same as wobj (identity in wobj frame)
            target_floor_transformation_matrix_in_wobj = np.eye(4)
            target_floor_transformation_matrix_in_wobj[:3, 3] = target_floor_origin_in_wobj  # New origin position
            
            # Extract frame axes (same as wobj axes in wobj frame, which are identity axes)
            target_floor_x_in_wobj = self.wobj_x_in_wobj.copy()
            target_floor_y_in_wobj = self.wobj_y_in_wobj.copy()
            target_floor_z_in_wobj = self.wobj_z_in_wobj.copy()
            
            result = {
                'target_floor_origin_in_wobj': target_floor_origin_in_wobj,
                'target_floor_x_in_wobj': target_floor_x_in_wobj,
                'target_floor_y_in_wobj': target_floor_y_in_wobj,
                'target_floor_z_in_wobj': target_floor_z_in_wobj,
                'target_floor_transformation_matrix_in_wobj': target_floor_transformation_matrix_in_wobj
            }
            
            print(f"Generated frame for floor index {index} (in wobj frame)")
            print(f"Target floor origin: {target_floor_origin_in_wobj}")
            print(f"Target floor X-axis: {target_floor_x_in_wobj}")
            print(f"Target floor Y-axis: {target_floor_y_in_wobj}")
            print(f"Target floor Z-axis: {target_floor_z_in_wobj}")
            print(f"Target floor transformation matrix in wobj:\n{target_floor_transformation_matrix_in_wobj}")
            
            return result
            
        except Exception as e:
            print(f"Error generating frame for target floor in wobj: {e}")
            return None
    
    def generate_target_floor_frame_in_base_with_offset(self, index=14, offset_x=0, offset_y=0, offset_z=0):
        """Generate coordinate frame for target floor with additional offsets in base frame.
        
        First generates the target floor frame based on index, then applies additional offsets.
        
        Args:
            index (int): Floor index
            offset_x (float): Additional offset along wobj X-axis in meters
            offset_y (float): Additional offset along wobj Y-axis in meters
            offset_z (float): Additional offset along wobj Z-axis in meters
            
        Returns:
            dict: Dictionary containing frame information in base coordinate system
                  Returns None if wobj information is not loaded
        """
        # First generate the target floor frame
        target_floor_frame = self.generate_target_floor_frame_in_base(index)
        if target_floor_frame is None:
            print("Failed to generate target floor frame, cannot apply offsets")
            return None
        
        try:
            # Get the target floor origin from the generated frame
            base_origin = target_floor_frame['target_floor_origin_in_base']
            
            # Apply additional offsets along wobj axes (in base frame)
            new_origin_in_base = (base_origin + 
                                  offset_x * self.wobj_x_in_base + 
                                  offset_y * self.wobj_y_in_base + 
                                  offset_z * self.wobj_z_in_base)
            
            # Create transformation matrix for the new frame (in base frame)
            # Pose (rotation) remains the same as wobj
            new_transformation_matrix_in_base = np.eye(4)
            new_transformation_matrix_in_base[:3, :3] = self.wobj_transformation_matrix_in_base[:3, :3]
            new_transformation_matrix_in_base[:3, 3] = new_origin_in_base
            
            # Extract frame axes (same as wobj axes, in base frame)
            new_x_in_base = self.wobj_x_in_base.copy()
            new_y_in_base = self.wobj_y_in_base.copy()
            new_z_in_base = self.wobj_z_in_base.copy()
            
            result = {
                'target_floor_origin_in_base': new_origin_in_base,
                'target_floor_x_in_base': new_x_in_base,
                'target_floor_y_in_base': new_y_in_base,
                'target_floor_z_in_base': new_z_in_base,
                'target_floor_transformation_matrix_in_base': new_transformation_matrix_in_base
            }
            
            print(f"Generated frame for floor index {index} with offsets (in base frame)")
            print(f"Additional offsets - X: {offset_x}, Y: {offset_y}, Z: {offset_z}")
            print(f"New origin: {new_origin_in_base}")
            print(f"New X-axis: {new_x_in_base}")
            print(f"New Y-axis: {new_y_in_base}")
            print(f"New Z-axis: {new_z_in_base}")
            print(f"New transformation matrix in base:\n{new_transformation_matrix_in_base}")
            
            return result
            
        except Exception as e:
            print(f"Error generating frame with offsets: {e}")
            return None


def main():
    """Main function to generate and print floor coordinate frame information."""
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Generate server frame for target floor')
    parser.add_argument('--index', type=int, default=14, 
                       help='Floor index (default: 14)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("Generate Server Frame by Wobj")
    print("=" * 60)
    
    # Create instance
    generator = GenerateServerFrame()
    
    # Load wobj information from service
    print("\n📥 Loading wobj coordinate system from robot_status service...")
    if not generator._load_wobj_from_service():
        print("❌ Failed to load wobj information")
        return
    
    # Generate frame for target floor in base frame
    print(f"\n🎯 Generating frame for floor index: {args.index}")
    print("-" * 60)
    
    print("\n📍 Frame in Base Coordinate System:")
    print("=" * 60)
    result_base = generator.generate_target_floor_frame_in_base(args.index)
    
    if result_base:
        print("\n✅ Frame in base generated successfully!")
    else:
        print("\n❌ Failed to generate frame in base")
        return
    
    # Generate frame for target floor in wobj frame
    print("\n" + "=" * 60)
    print("📍 Frame in Wobj Coordinate System:")
    print("=" * 60)
    result_wobj = generator.generate_target_floor_frame_in_wobj(args.index)
    
    if result_wobj:
        print("\n✅ Frame in wobj generated successfully!")
    else:
        print("\n❌ Failed to generate frame in wobj")
        return
    
    # Generate frame for target floor with offset
    print("\n" + "=" * 60)
    print("📍 Frame in Base Coordinate System with Y-offset:")
    print("=" * 60)
    result_with_offset = generator.generate_target_floor_frame_in_base_with_offset(
        args.index, offset_x=0.0, offset_y=-0.60, offset_z=0.0
    )
    
    if result_with_offset:
        print("\n✅ Frame with offset generated successfully!")
        print("=" * 60)
    else:
        print("\n❌ Failed to generate frame with offset")


if __name__ == "__main__":
    main()
