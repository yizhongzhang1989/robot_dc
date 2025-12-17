#!/usr/bin/env python3
"""
Demo Task Manager
Manages and coordinates tasks between courier robot and AMR control systems
"""

import sys
import os
from pathlib import Path

# Add current directory to path for imports
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

# Import the required modules
from courier_robot_webapi import CourierRobotWebAPI
from scripts.demo_amr_functions import AMRController


class TaskManager:
    """
    Task Manager class for coordinating robot operations
    Manages both courier robot and AMR control systems
    """
    
    def __init__(self):
        """
        Initialize Task Manager
        """
        self.courier_robot = None
        self.amr_controller = None
        
        # Initialize both systems using private methods
        self._init_courier_robot()
        self._init_amr_controller()
    
    def _init_courier_robot(self):
        """
        Private method to initialize courier robot web API
        """
        try:
            self.courier_robot = CourierRobotWebAPI()
            print("Courier Robot Web API initialized successfully")
        except Exception as e:
            print(f"Failed to initialize Courier Robot Web API: {e}")
            self.courier_robot = None
    
    def _init_amr_controller(self):
        """
        Private method to initialize AMR controller
        """
        try:
            self.amr_controller = AMRController()
            print("AMR Controller initialized successfully")
        except Exception as e:
            print(f"Failed to initialize AMR Controller: {e}")
            self.amr_controller = None


if __name__ == "__main__":
    # Create task manager instance for testing
    task_manager = TaskManager()
    print("Task Manager initialization completed")