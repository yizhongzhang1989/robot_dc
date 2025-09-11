#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os

def collect_joint_angles_from_temp():
    """
    Read all JSON files from temp folder and extract joint angle information,
    save to collect_point.json file
    """
    temp_folder = "/home/a/Documents/robot_dc2/colcon_ws/temp"
    output_file = os.path.join(temp_folder, "collect_points.json")
    
    collected_points = []
    
    # Get all numbered JSON files (exclude other special JSON files)
    json_files = []
    for i in range(100):  # assume max 100 files
        json_file = os.path.join(temp_folder, f"{i}.json")
        if os.path.exists(json_file):
            json_files.append(json_file)
    
    # Sort by filename to ensure correct order
    json_files.sort(key=lambda x: int(os.path.basename(x).split('.')[0]))
    
    print(f"Found {len(json_files)} JSON files")
    
    # Read each JSON file
    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # Extract joint_angles
            if 'joint_angles' in data:
                joint_angles = data['joint_angles']
                
                # Create point info record
                point_info = {
                    "point_id": int(os.path.basename(json_file).split('.')[0]),
                    "joint_angles": joint_angles
                }
                
                collected_points.append(point_info)
                print(f"Read file: {os.path.basename(json_file)}, Joint angles: {joint_angles}")
                
            else:
                print(f"Warning: 'joint_angles' field not found in {os.path.basename(json_file)}")
                
        except Exception as e:
            print(f"Error reading file {os.path.basename(json_file)}: {str(e)}")
    
    # Save result to collect_point.json
    result_data = {
        "total_points": len(collected_points),
        "points": collected_points
    }
    
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, indent=2, ensure_ascii=False)
        
        print(f"\nSuccessfully saved {len(collected_points)} data points to {output_file}")
        print(f"Total collected {len(collected_points)} joint angles")
        
    except Exception as e:
        print(f"Error saving file: {str(e)}")
    
    return collected_points

def print_collected_summary(collected_points):
    """
    Print summary of collected data
    """
    print("\n=== Data Collection Summary ===")
    print(f"Total data points: {len(collected_points)}")
    
    if collected_points:
        print(f"First point (ID: {collected_points[0]['point_id']}): {collected_points[0]['joint_angles']}")
        print(f"Last point (ID: {collected_points[-1]['point_id']}): {collected_points[-1]['joint_angles']}")

if __name__ == "__main__":
    print("Starting to collect joint angles data from temp folder...")
    collected_data = collect_joint_angles_from_temp()
    print_collected_summary(collected_data)
    print("Data collection completed!")