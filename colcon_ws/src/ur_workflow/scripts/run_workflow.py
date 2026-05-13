#!/usr/bin/env python3
"""
UR15 Workflow Runner - ROS 2 Package Entry Point

This script runs workflows from the ur_workflow ROS package.

Usage:
    ros2 run ur_workflow run_workflow.py --config <path_to_config>
    
    # Using package example files
    ros2 run ur_workflow run_workflow.py --config $(ros2 pkg prefix ur_workflow)/share/ur_workflow/examples/workflow_simple.yaml
"""

import sys
import os
from pathlib import Path

# Import and run the workflow runner
from ur_workflow.runner import main

if __name__ == "__main__":
    sys.exit(main())
