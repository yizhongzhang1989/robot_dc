#!/usr/bin/env python3

import os
import sys
import subprocess

def main():
    # Change to workspace directory
    workspace_dir = "/home/jetson/Desktop/robot_dc/colcon_ws"
    os.chdir(workspace_dir)
    
    # Source ROS2 environment
    setup_cmd = "source install/setup.bash"
    
    # Run the display node
    display_cmd = "python3 -m cam_node.cam_node_display"
    
    # Combine commands
    full_cmd = f"{setup_cmd} && {display_cmd}"
    
    print("Starting camera display node...")
    print("Commands to run:")
    print(f"cd {workspace_dir}")
    print(full_cmd)
    print("\nPress Ctrl+C to stop the node")
    print("In camera windows: Press 'q' to quit, 's' to save snapshot")
    
    try:
        # Run the command
        result = subprocess.run(full_cmd, shell=True, executable='/bin/bash')
        return result.returncode
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0
    except Exception as e:
        print(f"Error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
