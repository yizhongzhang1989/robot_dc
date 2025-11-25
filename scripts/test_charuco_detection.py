#!/usr/bin/env python3
"""
CharucoBoard Detection Test Script
================================

This script tests different ArUco dictionaries to find which one can detect
the CharucoBoard corners in your test image.
"""

import cv2
import numpy as np
import sys
import os

# Add ThirdParty path
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty'))

try:
    from camera_calibration_toolkit.core.calibration_patterns import get_pattern_manager
except ImportError as e:
    print(f"Error importing camera_calibration_toolkit: {e}")
    sys.exit(1)

def test_charuco_detection():
    """Test CharucoBoard detection with different dictionaries"""
    
    # Load test image
    test_image_path = "../temp/test/0.jpg"
    if not os.path.exists(test_image_path):
        print(f"Test image not found: {test_image_path}")
        return
    
    image = cv2.imread(test_image_path)
    if image is None:
        print(f"Failed to load image: {test_image_path}")
        return
    
    # Create output directory
    output_dir = "../temp/test/charuco_detect"
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Loaded test image: {test_image_path}")
    print(f"Image shape: {image.shape}")
    print(f"Output directory: {output_dir}")
    
    # Get pattern manager
    pattern_manager = get_pattern_manager()
    
    # Dictionary options to test
    dict_options = [
        (0, "DICT_4X4_50"),
        (1, "DICT_4X4_100"), 
        (2, "DICT_4X4_250"),
        (3, "DICT_4X4_1000"),
        (4, "DICT_5X5_50"),
        (5, "DICT_5X5_100"),
        (6, "DICT_5X5_250"),
        (7, "DICT_5X5_1000"),
        (8, "DICT_6X6_50"),
        (9, "DICT_6X6_100"),
        (10, "DICT_6X6_250"),
        (11, "DICT_6X6_1000"),
        (12, "DICT_7X7_50"),
        (13, "DICT_7X7_100"),
        (14, "DICT_7X7_250"),
        (15, "DICT_7X7_1000"),
        (16, "DICT_ARUCO_ORIGINAL")
    ]
    
    # CharucoBoard parameters (your ChArUco-400 specs)
    width = 12          # squares across width
    height = 9          # squares across height  
    square_size = 0.03  # 30mm
    marker_size = 0.0225 # 22.5mm
    
    print(f"\nTesting CharucoBoard detection:")
    print(f"- Board size: {width}x{height} squares")
    print(f"- Square size: {square_size}m ({square_size*1000}mm)")
    print(f"- Marker size: {marker_size}m ({marker_size*1000}mm)")
    print("=" * 60)
    
    successful_detections = []
    
    for dict_id, dict_name in dict_options:
        try:
            # Create CharucoBoard with this dictionary
            charuco_pattern = pattern_manager.create_pattern(
                'charuco_board',
                width=width,
                height=height,
                square_size=square_size,
                marker_size=marker_size,
                dictionary_id=dict_id
            )
            
            # Test detection
            ret, corners, point_ids = charuco_pattern.detect_corners(image)
            
            if ret and corners is not None:
                corner_count = len(corners)
                point_id_count = len(point_ids) if point_ids is not None else 0
                print(f"✅ {dict_name} (ID:{dict_id:2d}) - Detected {corner_count} corners, {point_id_count} IDs")
                successful_detections.append((dict_id, dict_name, corner_count, point_id_count))
                
                # Draw detection results and save image
                result_image = charuco_pattern.draw_corners(image.copy(), corners, point_ids)
                
                # Add text annotation
                text = f"{dict_name} - {corner_count} corners"
                cv2.putText(result_image, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                           1.5, (0, 255, 0), 3, cv2.LINE_AA)
                cv2.putText(result_image, f"Dictionary ID: {dict_id}", (30, 100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2, cv2.LINE_AA)
                
                # Save result image
                output_filename = f"charuco_detect_{dict_name}_id{dict_id:02d}_{corner_count}corners.jpg"
                output_path = os.path.join(output_dir, output_filename)
                cv2.imwrite(output_path, result_image)
                print(f"   💾 Saved: {output_filename}")
                
            else:
                print(f"❌ {dict_name} (ID:{dict_id:2d}) - No pattern detected")
                
        except Exception as e:
            print(f"⚠️  {dict_name} (ID:{dict_id:2d}) - Error: {e}")
    
    print("=" * 60)
    
    if successful_detections:
        print(f"\n🎯 SUCCESSFUL DETECTIONS ({len(successful_detections)} found):")
        for dict_id, dict_name, corner_count, point_id_count in successful_detections:
            print(f"   Dictionary: {dict_name} (ID: {dict_id})")
            print(f"   Corners: {corner_count}, Point IDs: {point_id_count}")
            print()
        
        # Recommend the best option
        best_detection = max(successful_detections, key=lambda x: x[2])  # Max corners
        dict_id, dict_name, corner_count, point_id_count = best_detection
        print(f"🏆 RECOMMENDED: {dict_name} (ID: {dict_id}) with {corner_count} corners")
        print(f"   Use this dictionary_id in your web interface.")
        
        print(f"\n💾 Detection results saved to: {output_dir}")
        print(f"   {len(successful_detections)} images with detected corners saved")
        
    else:
        print("❌ NO SUCCESSFUL DETECTIONS")
        print("   Possible issues:")
        print("   - Image doesn't contain a CharucoBoard")
        print("   - Board parameters don't match actual board")
        print("   - Image quality or lighting issues")
        print("   - Board is partially occluded or distorted")

if __name__ == "__main__":
    test_charuco_detection()