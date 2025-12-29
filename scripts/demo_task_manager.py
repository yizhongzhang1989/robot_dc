#!/usr/bin/env python3
"""
Demo Task Manager
Manages and coordinates tasks between courier robot and AMR control systems
"""

import sys
import os
import time
import subprocess
import threading
import argparse
import math
import yaml
from pathlib import Path

# Use common package for workspace utilities
try:
    from common.workspace_utils import get_workspace_root
    # Add scripts directory to path for imports
    workspace_root = get_workspace_root()
    if workspace_root:
        scripts_dir = os.path.join(workspace_root, 'scripts')
        sys.path.insert(0, scripts_dir)
    else:
        # Fallback to current directory method
        current_dir = Path(__file__).parent
        sys.path.insert(0, str(current_dir))
except ImportError:
    # Fallback if common package is not available
    current_dir = Path(__file__).parent
    sys.path.insert(0, str(current_dir))

# Import the required modules
from courier_robot_webapi import CourierRobotWebAPI
from demo_amr_functions import AMRController
from robot_status_redis.client_utils import RobotStatusClient
from ur_operate_tools import UROperateTools

class TaskManager:
    """
    Task Manager class for coordinating robot operations
    Manages both courier robot and AMR control systems
    """
    
    def __init__(self, server_index=None):
        """
        Initialize Task Manager
        
        Args:
            server_index (int, optional): Server index for identification in redis
        """
        self.courier_robot = None
        self.amr_controller = None
        self.ur_operate_tools = None
        self.server_index = None
        self.redis_client = None
        
        # Set up workspace paths
        self._set_paths()
        
        # Initialize redis client
        self._init_redis_client()
        
        # Set up server ID and upload to redis
        self._setup_server_id(server_index)
        
        # Initialize all systems using private methods
        self._init_courier_robot()
        self._init_amr_controller()
        self._init_ur_operate_tools()
    
    # ===================================== Initialization Method ======================================
    def _set_paths(self):
        """
        Private method to set up workspace directory paths using common package functions
        """
        try:
            from common.workspace_utils import (
                get_workspace_root, 
                get_scripts_directory,
                get_temp_directory
            )
            
            # Get workspace root directory (robot_dc)
            self.workspace_dir = get_workspace_root()
            if self.workspace_dir is None:
                raise RuntimeError("Could not determine workspace root directory")
            
            # Get scripts directory
            self.scripts_dir = get_scripts_directory()
            if self.scripts_dir is None:
                self.scripts_dir = os.path.join(self.workspace_dir, 'scripts')
            
            # Get temp directory (automatically creates if not exists)
            self.temp_dir = get_temp_directory()
            
            # Construct other directories based on workspace root
            self.dataset_dir = os.path.join(self.workspace_dir, 'dataset')
            self.config_dir = os.path.join(self.workspace_dir, 'config')
            
            # Create directories if they don't exist
            os.makedirs(self.dataset_dir, exist_ok=True)
            os.makedirs(self.config_dir, exist_ok=True)
            
            print(f"✓ Workspace paths initialized:")
            print(f"  - Workspace: {self.workspace_dir}")
            print(f"  - Scripts: {self.scripts_dir}")
            print(f"  - Dataset: {self.dataset_dir}")
            print(f"  - Temp: {self.temp_dir}")
            print(f"  - Config: {self.config_dir}")
            
        except ImportError as e:
            print(f"⚠ Warning: Could not import common package functions: {e}")
            # Fallback to relative path construction
            self._set_paths_fallback()
        except Exception as e:
            print(f"⚠ Warning: Error setting up workspace paths: {e}")
            # Fallback to relative path construction
            self._set_paths_fallback()
    
    def _set_paths_fallback(self):
        """
        Fallback method to set up paths when common package is not available
        """
        current_dir = Path(__file__).parent
        self.workspace_dir = current_dir.parent
        self.scripts_dir = str(current_dir)
        self.dataset_dir = os.path.join(self.workspace_dir, 'dataset')
        self.temp_dir = os.path.join(self.workspace_dir, 'temp')
        self.config_dir = os.path.join(self.workspace_dir, 'config')
        
        # Create directories if they don't exist
        os.makedirs(self.dataset_dir, exist_ok=True)
        os.makedirs(self.temp_dir, exist_ok=True)
        os.makedirs(self.config_dir, exist_ok=True)
        
        print(f"✓ Workspace paths initialized (fallback mode):")
        print(f"  - Workspace: {self.workspace_dir}")
        print(f"  - Scripts: {self.scripts_dir}")
        print(f"  - Dataset: {self.dataset_dir}")
        print(f"  - Temp: {self.temp_dir}")
        print(f"  - Config: {self.config_dir}")

    def _init_redis_client(self):
        """
        Private method to initialize robot status redis client
        """
        try:
            self.redis_client = RobotStatusClient()
            print(f"✓ Redis client initialized successfully")
        except ImportError as e:
            print(f"⚠ Warning: Could not import RobotStatusClient: {e}")
            self.redis_client = None
        except Exception as e:
            print(f"⚠ Warning: Error initializing redis client: {e}")
            self.redis_client = None

    def _setup_server_id(self, server_index):
        """
        Private method to setup server index and upload to robot_status_redis
        
        Args:
            server_index (int, optional): Server index for identification
        """
        # Store server_index directly
        self.server_index = server_index if server_index is not None else 0
            
        # Upload server_index to redis if client is available
        if self.redis_client is not None:
            try:
                # Store server_index directly in redis under ur15 namespace using set_status
                self.redis_client.set_status("ur15", "rack_operating_unit_id", self.server_index)
            
                print(f"✓ Server index setup completed:")
                print(f"  - Server Index: {self.server_index}")
                print(f"  - Data uploaded to redis: ur15:rack_operating_unit_id")
                
            except Exception as e:
                print(f"⚠ Warning: Error uploading server index to redis: {e}")
                print(f"✓ Server index set locally: {self.server_index}")
        else:
            print(f"⚠ Warning: Redis client not available")
            print(f"✓ Server index set locally: {self.server_index}")

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
    
    def _init_ur_operate_tools(self):
        """
        Private method to initialize UR operate tools
        """
        try:
            self.ur_operate_tools = UROperateTools()
            print("UR Operate Tools initialized successfully")
        except Exception as e:
            print(f"Failed to initialize UR Operate Tools: {e}")
            self.ur_operate_tools = None
    
    def _load_target_workflow_file(self):
        """
        Private method to load workflow file name from robot_config.yaml
        
        Returns:
            str: Workflow file name from config, or None if not found
        """
        try:
            # Build path to robot_config.yaml
            config_file = os.path.join(self.config_dir, 'robot_config.yaml')
            
            # Check if config file exists
            if not os.path.exists(config_file):
                print(f"✗ Config file not found: {config_file}")
                return None
            
            # Load yaml configuration
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Navigate to ur15 -> task -> rack_positioning -> workflow_name
            workflow_name = config.get('ur15', {}).get('task', {}).get('rack_positioning', {}).get('workflow_name')
            
            if workflow_name:
                print(f"✓ Workflow file loaded from config: {workflow_name}")
                return workflow_name
            else:
                print(f"✗ Workflow name not found in config at path: ur15.task.rack_positioning.workflow_name")
                return None
                
        except yaml.YAMLError as e:
            print(f"✗ Error parsing YAML config: {e}")
            return None
        except Exception as e:
            print(f"✗ Error loading workflow file from config: {e}")
            return None
    
    # Note: ur_operate_wobj functionality is now available through ur_operate_tools
    # since UROperateTools inherits from UROperateWobj

    # ===================================== Script Execution Method ======================================
    def _execute_ur_wobj_script(self, script_path, task_name):
        """
        Private method to execute ur_wobj_xxx.py scripts
        
        Args:
            script_path (str): Full path to the script file
            task_name (str): Name of the task for logging
            
        Returns:
            bool: True if script executed successfully, False otherwise
        """
        # Check if script file exists
        if not os.path.exists(script_path):
            print(f"✗ Script file not found: {script_path}")
            return False
        
        print(f"📋 Executing {task_name} task: {script_path}")
        print(f"🤖 Starting UR15 {task_name} operation...")
        
        try:
            # Build python3 command to execute the script with server_index
            command = ['/usr/bin/python3', script_path, '--server-index', str(self.server_index)]
            
            print(f"🚀 Running command: {' '.join(command)}")
            
            # Execute the script
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=self.scripts_dir  # Set working directory to scripts folder
            )
            
            print(f"⏳ {task_name} process started with PID: {process.pid}")
            print(f"📊 Executing {task_name} workflow...")
            
            # Wait for process to complete and get output
            stdout, stderr = process.communicate()
            return_code = process.returncode
            
            # Process results
            if return_code == 0:
                print(f"\\n✓ {task_name} task completed successfully!")
                if stdout.strip():
                    print(f"\\n📋 {task_name} output:")
                    print(stdout)
                return True
            else:
                print(f"\\n✗ {task_name} task failed with exit code: {return_code}")
                if stderr:
                    print(f"❌ Error details:\\n{stderr}")
                if stdout:
                    print(f"📋 {task_name} output:\\n{stdout}")
                return False
                
        except FileNotFoundError:
            print(f"✗ Error: Python3 command not found for {task_name} task.")
            return False
        except Exception as e:
            print(f"✗ Error executing {task_name} task: {e}")
            return False
    
    # ====================================== Operation Methods ======================================
    def ur15_execute_rack_positioning_task(self):
        """
        Execute rack positioning workflow using UR15 robot
        """

        # Load workflow filename from config
        workflow_filename = self._load_target_workflow_file()
        if workflow_filename is None:
            print("✗ Failed to load workflow filename from config")
            return False
        
        # Define workflow file path using temp directory
        workflow_file = os.path.join(self.temp_dir, "workflow_files", workflow_filename)
        
        # Check if workflow file exists
        if not os.path.exists(workflow_file):
            print(f"✗ Workflow file not found: {workflow_file}")
            return False
        
        print(f"📋 Executing workflow: {workflow_file}")
        print("🤖 Starting UR15 rack positioning task...")
        
        try:
            # Build ros2 run command
            command = ['ros2', 'run', 'ur15_workflow', 'run_workflow.py', '--config', workflow_file]
            
            print(f"🚀 Running command: {' '.join(command)}")
            
            # Execute the workflow
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            print(f"⏳ Workflow process started with PID: {process.pid}")
            print("📊 Executing rack positioning workflow...")
            
            # Wait for process to complete and get output
            stdout, stderr = process.communicate()
            return_code = process.returncode
            
            # Process results
            if return_code == 0:
                print("\n✓ Rack positioning workflow completed successfully!")                
                if stdout.strip():
                    print("\n📋 Workflow output:")
                    print(stdout)
                
                return True
            else:
                print(f"\n✗ Workflow failed with exit code: {return_code}")
                if stderr:
                    print(f"❌ Error details:\n{stderr}")
                if stdout:
                    print(f"📋 Workflow output:\n{stdout}")
                return False
                
        except FileNotFoundError:
            print("✗ Error: ros2 command not found. Make sure ROS2 is installed and sourced.")
            return False
        except Exception as e:
            print(f"✗ Error executing workflow: {e}")
            return False
    
    def ur15_execute_unlock_knob_task(self):
        """
        Execute unlock_knob task using ur_wobj_unlock_knob.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_unlock_knob.py")
        return self._execute_ur_wobj_script(script_path, "unlock_knob")
    
    def ur15_execute_open_handle_task(self):
        """
        Execute open_handle task using ur_wobj_open_handle.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_open_handle.py")
        return self._execute_ur_wobj_script(script_path, "open_handle")

    def ur15_execute_close_left_task(self):
        """
        Execute close_left task using ur_wobj_close_left.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_close_left.py")
        return self._execute_ur_wobj_script(script_path, "close_left")
    
    def ur15_execute_close_right_task(self):
        """
        Execute close_right task using ur_wobj_close_right.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_close_right.py")
        return self._execute_ur_wobj_script(script_path, "close_right")

    def ur15_execute_extract_server_task(self):
        """
        Execute extract_server task using ur_wobj_extract_server.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_extract_server.py")
        return self._execute_ur_wobj_script(script_path, "extract_server")
    
    def ur15_execute_put_frame_task(self):
        """
        Execute put_frame task using ur_wobj_put_frame.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_put_frame.py")
        return self._execute_ur_wobj_script(script_path, "put_frame")
    
    def ur15_execute_insert_server_task(self):
        """
        Execute insert_server task using ur_wobj_insert_server.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_insert_server.py")
        return self._execute_ur_wobj_script(script_path, "insert_server")
    
    def ur15_execute_unlock_knob_insert_task(self):
        """
        Execute unlock_knob_insert task using ur_wobj_unlock_knob_insert.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_unlock_knob_insert.py")
        return self._execute_ur_wobj_script(script_path, "unlock_knob_insert")
    
    def ur15_execute_close_handles_task(self):
        """
        Execute close_handles task using ur_wobj_close_handles.py script
        """
        script_path = os.path.join(self.scripts_dir, "ur_wobj_close_handles.py")
        return self._execute_ur_wobj_script(script_path, "close_handles")

    def ur15_execute_movej_to_home_position(self):
        """
        Execute UR15 movej to home position by moving each joint in reverse order (J6 to J1).
        
        The robot will move:
        1. Joint 6 (J6) first
        2. Joint 5 (J5)
        3. Joint 4 (J4)
        4. Joint 3 (J3)
        5. Joint 2 (J2)
        6. Joint 1 (J1) last
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.ur_operate_tools is None:
            print("✗ Error: UR Operate Tools not initialized. Cannot execute move operation.")
            return False
        
        print("=" * 60)
        print("Starting movej_to_home_position - Moving joints in order J6 -> J1")
        print("=" * 60)
        
        try:
            # Home position in degrees (target position)
            home_position_deg = [96.3, -92.3, 4.0, -98.1, -4.3, 178.0]
            print(f"Target home position (degrees): {home_position_deg}")
            
            # Get current joint positions
            current_position_rad = self.ur_operate_tools.robot.get_actual_joint_positions()
            current_position_deg = [math.degrees(angle) for angle in current_position_rad]
            print(f"Current position (degrees): {[f'{x:.2f}' for x in current_position_deg]}")
            print()
            
            # Movement parameters
            a_movej = 1.0  # Acceleration for joint movements (rad/s²)
            v_movej = 1.0  # Velocity for joint movements (rad/s)
            
            # Move each joint in reverse order (J6 to J1)
            # Joint indices: J1=0, J2=1, J3=2, J4=3, J5=4, J6=5
            joint_order = [5, 4, 3, 2, 1, 0]  # J6, J5, J4, J3, J2, J1
            joint_names = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1']
            
            # Create intermediate position starting from current position
            intermediate_position_deg = current_position_deg.copy()
            
            for idx, (joint_idx, joint_name) in enumerate(zip(joint_order, joint_names), 1):
                # Update the target joint position
                intermediate_position_deg[joint_idx] = home_position_deg[joint_idx]
                
                # Convert to radians for robot command
                intermediate_position_rad = [math.radians(angle) for angle in intermediate_position_deg]
                
                print(f"  Step {idx}/6: Moving {joint_name} to {home_position_deg[joint_idx]:.2f}°")
                
                # Execute joint movement
                result = self.ur_operate_tools.robot.movej(intermediate_position_rad, a=a_movej, v=v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move {joint_name}, error code: {result}")
                    return False
                
                print(f"  ✓ Successfully moved {joint_name}")
                
                # Add small delay between movements for stability
                time.sleep(0.3)
            
            # Verify final position
            final_position_rad = self.ur_operate_tools.robot.get_actual_joint_positions()
            final_position_deg = [math.degrees(angle) for angle in final_position_rad]
            print(f"\nFinal position (degrees): {[f'{x:.2f}' for x in final_position_deg]}")
            
            # Calculate position error
            position_error = [abs(final - target) for final, target in zip(final_position_deg, home_position_deg)]
            print(f"Position error (degrees): {[f'{x:.2f}' for x in position_error]}")
            
            print("=" * 60)
            print("✓ Successfully completed movej_to_home_position")
            print("=" * 60)
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_to_home_position: {e}")
            return False

    def ur15_execute_movej_to_task_position(self):
        """
        Execute UR15 movej to task position by moving each joint in reverse order (J6 to J1).
        
        The robot will move:
        1. Joint 6 (J6) first
        2. Joint 5 (J5)
        3. Joint 4 (J4)
        4. Joint 3 (J3)
        5. Joint 2 (J2)
        6. Joint 1 (J1) last
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.ur_operate_tools is None:
            print("✗ Error: UR Operate Tools not initialized. Cannot execute move operation.")
            return False
        
        print("=" * 60)
        print("Starting movej_to_task_position - Moving joints in order J6 -> J1")
        print("=" * 60)
        
        try:
            # Task position in degrees (target position)
            task_position_deg = [92.0, -57.5, 102.9, -47.9, 9.6, 187.1]
            print(f"Target task position (degrees): {task_position_deg}")
            
            # Get current joint positions
            current_position_rad = self.ur_operate_tools.robot.get_actual_joint_positions()
            current_position_deg = [math.degrees(angle) for angle in current_position_rad]
            print(f"Current position (degrees): {[f'{x:.2f}' for x in current_position_deg]}")
            print()
            
            # Movement parameters
            a_movej = 1.0  # Acceleration for joint movements (rad/s²)
            v_movej = 1.0  # Velocity for joint movements (rad/s)
            
            # Move each joint in reverse order (J6 to J1)
            # Joint indices: J1=0, J2=1, J3=2, J4=3, J5=4, J6=5
            joint_order = [5, 4, 3, 2, 1, 0]  # J6, J5, J4, J3, J2, J1
            joint_names = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1']
            
            # Create intermediate position starting from current position
            intermediate_position_deg = current_position_deg.copy()
            
            for idx, (joint_idx, joint_name) in enumerate(zip(joint_order, joint_names), 1):
                # Update the target joint position
                intermediate_position_deg[joint_idx] = task_position_deg[joint_idx]
                
                # Convert to radians for robot command
                intermediate_position_rad = [math.radians(angle) for angle in intermediate_position_deg]
                
                print(f"  Step {idx}/6: Moving {joint_name} to {task_position_deg[joint_idx]:.2f}°")
                
                # Execute joint movement
                result = self.ur_operate_tools.robot.movej(intermediate_position_rad, a=a_movej, v=v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move {joint_name}, error code: {result}")
                    return False
                
                print(f"  ✓ Successfully moved {joint_name}")
                
                # Add small delay between movements for stability
                time.sleep(0.3)
            
            # Verify final position
            final_position_rad = self.ur_operate_tools.robot.get_actual_joint_positions()
            final_position_deg = [math.degrees(angle) for angle in final_position_rad]
            print(f"\nFinal position (degrees): {[f'{x:.2f}' for x in final_position_deg]}")
            
            # Calculate position error
            position_error = [abs(final - target) for final, target in zip(final_position_deg, task_position_deg)]
            print(f"Position error (degrees): {[f'{x:.2f}' for x in position_error]}")
            
            print("=" * 60)
            print("✓ Successfully completed movej_to_task_position")
            print("=" * 60)
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_to_task_position: {e}")
            return False

    def _print_execution_summary(self, step_status):
        """
        Print execution summary showing which steps succeeded and which failed
        
        Args:
            step_status: Dictionary containing step names and their execution status
        """
        print("\n" + "=" * 80)
        print("📊 EXECUTION SUMMARY")
        print("=" * 80)
        
        success_count = 0
        failed_count = 0
        skipped_count = 0
        pending_count = 0
        
        for step_name, status in step_status.items():
            if status == "SUCCESS":
                print(f"✓ {step_name}: {status}")
                success_count += 1
            elif status == "FAILED":
                print(f"✗ {step_name}: {status}")
                failed_count += 1
            elif status == "SKIPPED":
                print(f"⊘ {step_name}: {status}")
                skipped_count += 1
            else:  # PENDING
                print(f"⊙ {step_name}: {status}")
                pending_count += 1
        
        print("-" * 80)
        print(f"Total Steps: {len(step_status)}")
        print(f"✓ Successful: {success_count}")
        print(f"✗ Failed: {failed_count}")
        print(f"⊘ Skipped: {skipped_count}")
        print(f"⊙ Pending: {pending_count}")
        print("=" * 80)

    def execute_complete_task_sequence(self):
        """
        Execute the complete task sequence including AMR arm movement and UR15 positioning
        
        Returns:
            bool: True if entire sequence completed successfully, False otherwise
        """
        print("🚀 Starting complete task sequence execution...")
        print("=" * 60)
        
        # Initialize step execution tracking
        step_status = {
            "Step 0: AMR from home to arm dock": "PENDING",
            "Step 1: AMR move arms from DOCK to SIDE": "PENDING",
            "Step 2: UR15 rack positioning": "PENDING",
            "Step 3: UR15 get tool_rotate": "PENDING",
            "Step 4: UR15 unlock knobs": "PENDING",
            "Step 5: UR15 tool exchange (rotate→pushpull)": "PENDING",
            "Step 6: UR15 open handles": "PENDING",
            "Step 7: UR15 close left handle": "PENDING",
            "Step 8: UR15 close right handle": "PENDING",
            "Step 9: UR15 tool exchange (pushpull→extract)": "PENDING",
            "Step 10: UR15 move to target position": "PENDING",
            "Step 11: AMR courier to extraction position": "PENDING",
            "Step 12: UR15 extract server": "PENDING",
            "Step 13: AMR courier back to dock": "PENDING",
            "Step 14: UR15 return tool_extract & get frame": "PENDING",
            "Step 15: UR15 put frame": "PENDING",
            "Step 16: UR15 get tool_extract": "PENDING",
            "Step 17: UR15 move to target position": "PENDING",
            "Step 18: AMR courier to insertion position": "PENDING",
            "Step 19: UR15 insert server": "PENDING",
            "Step 20: AMR courier back to dock": "PENDING",
            "Step 21: UR15 tool exchange (extract→rotate)": "PENDING",
            "Step 22: UR15 unlock knob insert": "PENDING",
            "Step 23: UR15 tool exchange (rotate→pushpull)": "PENDING",
            "Step 24: UR15 close handles": "PENDING",
            "Step 25: UR15 return tool_pushpull": "PENDING",
            "Step 26: UR15 move to target position": "PENDING",
            "Step 27: AMR arm from side to dock": "PENDING",
            "Step 28: AMR back to home position": "PENDING",
        }
        
        # Check if all required components are initialized
        if self.amr_controller is None:
            print("✗ Error: AMR Controller not initialized. Cannot execute workflow.")
            self._print_execution_summary(step_status)
            return False
        if self.ur_operate_tools is None:
            print("✗ Error: UR Operate Tools not initialized. Cannot execute workflow.")
            self._print_execution_summary(step_status)
            return False
            
        # # ========================================================================
        # # STEP 0: AMR move from home to arm dock
        # # ========================================================================
        # print("\n📌 Step 0: Moving AMR from home to arm dock")
        # print("-" * 40)
        
        # try:
        #     # Execute first goto: LM2
        #     print("   → Moving to LM2...")
        #     result_lm2 = self.amr_controller.amr_controller.goto(target_id="LM2", wait=True)
            
        #     if not result_lm2.get('success', False):
        #         print("✗ Failed to move to LM2")
        #         step_status["Step 0: AMR from home to arm dock"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
        #     print("✓ Successfully Reached LM2")
            
        #     # Execute second goto: LM9
        #     print("   → Moving to LM9...")
        #     result_lm9 = self.amr_controller.amr_controller.goto(target_id="LM9", wait=True)
            
        #     if not result_lm9.get('success', False):
        #         print("✗ Failed to move to LM9")
        #         step_status["Step 0: AMR from home to arm dock"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
            
        #     print("✓ Successfully reached arm dock (LM9)")
        #     step_status["Step 0: AMR from home to arm dock"] = "SUCCESS"
                
        # except Exception as e:
        #     print(f"✗ Error during AMR navigation to arm dock: {e}")
        #     step_status["Step 0: AMR from home to arm dock"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
            
        # # ========================================================================
        # # STEP 1: AMR move robot arms from DOCK to SIDE position
        # # ========================================================================
        # print("\n📌 Step 1: Moving AMR arm from dock to side position")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR arm movement
        #     amr_result = self.amr_controller.amr_move_arm_from_dock_to_side()
            
        #     if amr_result:
        #         print("✓ AMR arm movement completed successfully")
        #         step_status["Step 1: AMR move arms from DOCK to SIDE"] = "SUCCESS"
        #     else:
        #         print("✗ AMR arm movement failed")
        #         step_status["Step 1: AMR move arms from DOCK to SIDE"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR arm movement: {e}")
        #     step_status["Step 1: AMR move arms from DOCK to SIDE"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
               
        # # ========================================================================
        # # STEP 2: UR15 execute rack positioning task
        # # ========================================================================
        # print("\n📌 Step 2: Executing UR15 rack positioning task")
        # print("-" * 40)
        
        # try:
        #     # First, move to task position using movej
        #     print("\n  → Moving to task position before rack positioning...")
        #     movej_task_result = self.ur15_execute_movej_to_task_position()
            
        #     if not movej_task_result:
        #         print("✗ UR15 movej to task position failed")
        #         step_status["Step 2: UR15 rack positioning"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
            
        #     print("✓ UR15 movej to task position completed successfully")
        #     print("\n  → Starting rack positioning workflow...")
            
        #     # Execute UR15 positioning workflow
        #     ur15_result = self.ur15_execute_rack_positioning_task()
            
        #     if ur15_result:
        #         print("✓ UR15 rack positioning task completed successfully")
        #         step_status["Step 2: UR15 rack positioning"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 rack positioning task failed")
        #         step_status["Step 2: UR15 rack positioning"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 positioning task: {e}")
        #     step_status["Step 2: UR15 rack positioning"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 3: Move UR15 to get tool_rotate
        # # ========================================================================
        # print("\n📌 Step 3: Executing UR15 tool operation - get tool_rotate")
        # print("-" * 40)
        
        # # Check if UR operate tools is initialized
        # if self.ur_operate_tools is None:
        #     print("✗ Error: UR Operate Tools not initialized. Cannot execute tool operation.")
        #     return False
        
        # try:
        #     # Execute get tool_rotate operation
        #     tool_result = self.ur_operate_tools.get_tool_from_task_position("tool_rotate")
            
        #     if tool_result:
        #         print("✓ UR15 tool_rotate operation completed successfully")
        #         step_status["Step 3: UR15 get tool_rotate"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 tool_rotate operation failed")
        #         step_status["Step 3: UR15 get tool_rotate"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool operation: {e}")
        #     step_status["Step 3: UR15 get tool_rotate"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 4: UR15 use FTC to unlock knobs on the server
        # # ========================================================================
        # print("\n📌 Step 4: Executing UR15 unlock knob task")
        # print("-" * 40)
        
        # try:
        #     # Execute unlock knob operation
        #     unlock_result = self.ur15_execute_unlock_knob_task()
            
        #     if unlock_result:
        #         print("✓ UR15 unlock knob task completed successfully")
        #         step_status["Step 4: UR15 unlock knobs"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 unlock knob task failed")
        #         step_status["Step 4: UR15 unlock knobs"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 unlock knob task: {e}")
        #     step_status["Step 4: UR15 unlock knobs"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 5: Move UR15 to return tool_rotate and then get tool_pushpull
        # # ========================================================================
        # print("\n📌 Step 5: Executing UR15 tool exchange operation - return tool_rotate and get tool_pushpull")
        # print("-" * 40)
        
        # # Check if UR operate tools is initialized
        # if self.ur_operate_tools is None:
        #     print("✗ Error: UR Operate Tools not initialized. Cannot execute tool exchange operation.")
        #     return False
        
        # try:
        #     # Execute return tool_rotate and get tool_pushpull operation
        #     exchange_result = self.ur_operate_tools.return_tool1_get_tool2_from_task(tool1_name="tool_rotate", tool2_name="tool_pushpull")
            
        #     if exchange_result:
        #         print("✓ UR15 tool exchange operation completed successfully")
        #         step_status["Step 5: UR15 tool exchange (rotate→pushpull)"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 tool exchange operation failed")
        #         step_status["Step 5: UR15 tool exchange (rotate→pushpull)"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool exchange operation: {e}")
        #     step_status["Step 5: UR15 tool exchange (rotate→pushpull)"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 6: UR15 use FTC to open handles on the server and then pull out the server for about 5 cm
        # # ========================================================================
        # print("\n📌 Step 6: Executing UR15 open handle task")
        # print("-" * 40)
        
        # try:
        #     # Execute open handle operation
        #     open_handle_result = self.ur15_execute_open_handle_task()
            
        #     if open_handle_result:
        #         print("✓ UR15 open handle task completed successfully")
        #         step_status["Step 6: UR15 open handles"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 open handle task failed")
        #         step_status["Step 6: UR15 open handles"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 open handle task: {e}")
        #     step_status["Step 6: UR15 open handles"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 7: UR15 re-positioning the position of server and then use FTC to close left handle
        # # ========================================================================
        # print("\n📌 Step 7: Executing UR15 close left task")
        # print("-" * 40)
        
        # try:
        #     # Execute close left operation
        #     close_left_result = self.ur15_execute_close_left_task()
            
        #     if close_left_result:
        #         print("✓ UR15 close left task completed successfully")
        #         step_status["Step 7: UR15 close left handle"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 close left task failed")
        #         step_status["Step 7: UR15 close left handle"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 close left task: {e}")
        #     step_status["Step 7: UR15 close left handle"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 8: UR15 re-positioning the position of server and then use FTC to close right handle
        # # ========================================================================
        # print("\n📌 Step 8: Executing UR15 close right task")
        # print("-" * 40)
        
        # try:
        #     # Execute close right operation
        #     close_right_result = self.ur15_execute_close_right_task()
            
        #     if close_right_result:
        #         print("✓ UR15 close right task completed successfully")
        #         step_status["Step 8: UR15 close right handle"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 close right task failed")
        #         step_status["Step 8: UR15 close right handle"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 close right task: {e}")
        #     step_status["Step 8: UR15 close right handle"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
                
        # # ========================================================================
        # # STEP 9: Move UR15 to return tool_pushpull and then get tool_extract
        # # ========================================================================
        # print("\n📌 Step 9: Executing UR15 tool exchange operation - return tool_pushpull and get tool_extract")
        # print("-" * 40)
        
        # # Check if UR operate tools is initialized
        # if self.ur_operate_tools is None:
        #     print("✗ Error: UR Operate Tools not initialized. Cannot execute tool exchange operation.")
        #     return False
        
        # try:
        #     # Execute return tool_pushpull and get tool_extract operation
        #     exchange_result = self.ur_operate_tools.return_tool1_get_tool2_from_task(tool1_name="tool_pushpull", tool2_name="tool_extract")
            
        #     if exchange_result:
        #         print("✓ UR15 tool exchange operation completed successfully")
        #         step_status["Step 9: UR15 tool exchange (pushpull→extract)"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 tool exchange operation failed")
        #         step_status["Step 9: UR15 tool exchange (pushpull→extract)"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool exchange operation: {e}")
        #     step_status["Step 9: UR15 tool exchange (pushpull→extract)"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 10: Move UR15 to target positions to avoid collision
        # # ========================================================================
        # print("\n📌 Step 10: Moving UR15 to target position")
        # print("-" * 40)
        
        # # Check if UR operate tools is initialized
        # if self.ur_operate_tools is None:
        #     print("✗ Error: UR Operate Tools not initialized. Cannot execute move operation.")
        #     return False
        
        # try:
        #     # Execute move to target position operation
        #     move_result = self.ur_operate_tools.movel_to_target_position(
        #         index=self.server_index,
        #         execution_order=[1, 3, 2],
        #         offset_in_rack=[0, -0.65, 0.45]
        #     )
            
        #     if not move_result:
        #         print("✓ UR15 move to target position completed successfully")
        #     else:
        #         print("✗ UR15 move to target position failed")
        #         step_status["Step 10: UR15 move to target position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
            
        #     # After movel_to_target_position, execute movej to specified joint angles
        #     print("\n📌 Step 10: Executing UR15 movej to target joint angles")
        #     print("-" * 40)
            
        #     target_joints_degrees = [113.2, -62.4, 65.1, -92.5, -90.3, -61.8]
        #     target_joints_radians = [math.radians(angle) for angle in target_joints_degrees]
            
        #     print(f"Target joint angles (degrees): {target_joints_degrees}")
        #     print(f"Target joint angles (radians): {[f'{rad:.4f}' for rad in target_joints_radians]}")
            
        #     # Execute movej operation
        #     movej_result = self.ur_operate_tools.robot.movej(
        #         target_joints_radians,
        #         a=0.5,  # acceleration
        #         v=0.5   # velocity
        #     )
            
        #     if movej_result == 0:
        #         print("✓ UR15 movej to target joint angles completed successfully")
        #         step_status["Step 10: UR15 move to target position"] = "SUCCESS"
        #     else:
        #         print(f"✗ UR15 movej to target joint angles failed with error code: {movej_result}")
        #         step_status["Step 10: UR15 move to target position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 move to target position: {e}")
        #     step_status["Step 10: UR15 move to target position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 11: AMR move courier robot from DOCK position to extraction position
        # # ========================================================================
        # print("\n📌 Step 11: Executing AMR courier movement to extraction position")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR courier movement
        #     courier_result = self.amr_controller.amr_move_courier_from_dock_to_extraction_position()
            
        #     if courier_result and courier_result.get('success', False):
        #         print("✓ AMR courier movement to extraction position completed successfully")
        #         step_status["Step 11: AMR courier to extraction position"] = "SUCCESS"
        #     else:
        #         print("✗ AMR courier movement to extraction position failed")
        #         step_status["Step 11: AMR courier to extraction position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR courier movement: {e}")
        #     step_status["Step 11: AMR courier to extraction position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 12: UR15 positioning the handles and then use FTC to extract the server from the rack
        # # ========================================================================
        # print("\n📌 Step 12: Executing UR15 extract server task")
        # print("-" * 40)
        
        # try:
        #     # Execute extract server operation
        #     extract_result = self.ur15_execute_extract_server_task()
            
        #     if extract_result:
        #         print("✓ UR15 extract server task completed successfully")
        #         step_status["Step 12: UR15 extract server"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 extract server task failed")
        #         step_status["Step 12: UR15 extract server"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 extract server task: {e}")
        #     step_status["Step 12: UR15 extract server"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 13: AMR move courier robot from extraction position back to DOCK position
        # # ========================================================================
        # print("\n📌 Step 13: Executing AMR courier movement from extraction position to dock")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR courier movement back to dock
        #     courier_return_result = self.amr_controller.amr_move_courier_from_extraction_position_to_dock()
            
        #     if courier_return_result and courier_return_result.get('success', False):
        #         print("✓ AMR courier movement from extraction position to dock completed successfully")
        #         step_status["Step 13: AMR courier back to dock"] = "SUCCESS"
        #     else:
        #         print("✗ AMR courier movement from extraction position to dock failed")
        #         step_status["Step 13: AMR courier back to dock"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR courier return movement: {e}")
        #     step_status["Step 13: AMR courier back to dock"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 14: Move UR15 to return tool_extract and then get tool_frame
        # # ========================================================================
        # print("\n📌 Step 14: Executing UR15 return tool_extract and get tool_frame")
        # print("-" * 40)
        
        # try:
        #     # Execute tool return and get frame operation using ur_operate_tools
        #     if self.ur_operate_tools:
        #         tool_return_result = self.ur_operate_tools.return_tool_get_frame_from_task(tool_name="tool_extract")
                
        #         if tool_return_result:
        #             print("✓ UR15 tool_extract return and frame tool get completed successfully")
        #             step_status["Step 14: UR15 return tool_extract & get frame"] = "SUCCESS"
        #         else:
        #             print("✗ UR15 tool_extract return and frame tool get failed")
        #             step_status["Step 14: UR15 return tool_extract & get frame"] = "FAILED"
        #             self._print_execution_summary(step_status)
        #             return False
        #     else:
        #         print("✗ UR operate tools not initialized")
        #         step_status["Step 14: UR15 return tool_extract & get frame"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool return operation: {e}")
        #     step_status["Step 14: UR15 return tool_extract & get frame"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 15: UR15 use FTC to put the frame onto the rack
        # # ========================================================================
        # print("\n📌 Step 15: Executing UR15 put frame task")
        # print("-" * 40)
        
        # try:
        #     # Execute put frame operation
        #     put_frame_result = self.ur15_execute_put_frame_task()
            
        #     if put_frame_result:
        #         print("✓ UR15 put frame task completed successfully")
        #         step_status["Step 15: UR15 put frame"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 put frame task failed")
        #         step_status["Step 15: UR15 put frame"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 put frame task: {e}")
        #     step_status["Step 15: UR15 put frame"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 16: Move UR15 to get tool_extract
        # # ========================================================================
        # print("\n📌 Step 16: Executing UR15 get tool_extract from task position")
        # print("-" * 40)
        
        # try:
        #     # Execute tool get operation using ur_operate_tools
        #     if self.ur_operate_tools:
        #         tool_get_result = self.ur_operate_tools.get_tool_from_task_position("tool_extract")
                
        #         if tool_get_result:
        #             print("✓ UR15 tool_extract get from storage completed successfully")
        #             step_status["Step 16: UR15 get tool_extract"] = "SUCCESS"
        #         else:
        #             print("✗ UR15 tool_extract get from storage failed")
        #             step_status["Step 16: UR15 get tool_extract"] = "FAILED"
        #             self._print_execution_summary(step_status)
        #             return False
        #     else:
        #         print("✗ UR operate tools not initialized")
        #         step_status["Step 16: UR15 get tool_extract"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool get operation: {e}")
        #     step_status["Step 16: UR15 get tool_extract"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 17: Move UR15 to target positions to avoid collision
        # # ========================================================================
        # print("\n📌 Step 17: Executing UR15 move to target position")
        # print("-" * 40)
        
        # # Check if UR operate tools is initialized
        # if self.ur_operate_tools is None:
        #     print("✗ Error: UR Operate Tools not initialized. Cannot execute move operation.")
        #     return False
        
        # try:
        #     # Execute move to target position operation
        #     move_result = self.ur_operate_tools.movel_to_target_position(
        #         index=self.server_index,
        #         execution_order=[1, 3, 2],
        #         offset_in_rack=[0, -0.65, 0.45]
        #     )
            
        #     if not move_result:
        #         print("✓ UR15 move to target position completed successfully")
        #     else:
        #         print("✗ UR15 move to target position failed")
        #         step_status["Step 17: UR15 move to target position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False

        #     # After movel_to_target_position, execute movej to specified joint angles
        #     print("\n📌 Step 10: Executing UR15 movej to target joint angles")
        #     print("-" * 40)
            
        #     target_joints_degrees = [113.2, -62.4, 65.1, -92.5, -90.3, -61.8]
        #     target_joints_radians = [math.radians(angle) for angle in target_joints_degrees]
            
        #     print(f"Target joint angles (degrees): {target_joints_degrees}")
        #     print(f"Target joint angles (radians): {[f'{rad:.4f}' for rad in target_joints_radians]}")
            
        #     # Execute movej operation
        #     movej_result = self.ur_operate_tools.robot.movej(
        #         target_joints_radians,
        #         a=1.0,  # acceleration
        #         v=1.0   # velocity
        #     )
            
        #     if movej_result == 0:
        #         print("✓ UR15 movej to target joint angles completed successfully")
        #         step_status["Step 17: UR15 move to target position"] = "SUCCESS"
        #     else:
        #         print(f"✗ UR15 movej to target joint angles failed with error code: {movej_result}")
        #         step_status["Step 17: UR15 move to target position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 move to target position: {e}")
        #     step_status["Step 17: UR15 move to target position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 18: AMR move courier robot from DOCK position to insertion position
        # # ========================================================================
        # print("\n📌 Step 18: Executing AMR courier movement from dock to insertion position")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR courier movement to insertion position
        #     courier_insertion_result = self.amr_controller.amr_move_courier_from_dock_to_insertion_position()
            
        #     if courier_insertion_result and courier_insertion_result.get('success', False):
        #         print("✓ AMR courier movement from dock to insertion position completed successfully")
        #         step_status["Step 18: AMR courier to insertion position"] = "SUCCESS"
        #     else:
        #         print("✗ AMR courier movement from dock to insertion position failed")
        #         step_status["Step 18: AMR courier to insertion position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR courier insertion movement: {e}")
        #     step_status["Step 18: AMR courier to insertion position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # ========================================================================
        # STEP 19: UR15 positioning the handles and then use FTC to insert the server into the rack
        # ========================================================================
        print("\n📌 Step 19: Executing UR15 insert server task")
        print("-" * 40)
        
        try:
            # Execute insert server operation
            insert_result = self.ur15_execute_insert_server_task()
            
            if insert_result:
                print("✓ UR15 insert server task completed successfully")
                step_status["Step 19: UR15 insert server"] = "SUCCESS"
            else:
                print("✗ UR15 insert server task failed")
                step_status["Step 19: UR15 insert server"] = "FAILED"
                self._print_execution_summary(step_status)
                return False
                
        except Exception as e:
            print(f"✗ Error during UR15 insert server task: {e}")
            step_status["Step 19: UR15 insert server"] = "FAILED"
            self._print_execution_summary(step_status)
            return False
        
        # # ========================================================================
        # # STEP 20: AMR move courier robot from insertion position back to DOCK position
        # # ========================================================================
        # print("\n📌 Step 20: Executing AMR courier movement from insertion position to dock")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR courier movement back to dock from insertion position
        #     courier_return_result = self.amr_controller.amr_move_courier_from_insertion_position_to_dock()
            
        #     if courier_return_result and courier_return_result.get('success', False):
        #         print("✓ AMR courier movement from insertion position to dock completed successfully")
        #         step_status["Step 20: AMR courier back to dock"] = "SUCCESS"
        #     else:
        #         print("✗ AMR courier movement from insertion position to dock failed")
        #         step_status["Step 20: AMR courier back to dock"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR courier return movement: {e}")
        #     step_status["Step 20: AMR courier back to dock"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False

        # # ========================================================================
        # # Here, we need to return tool_frame manually before proceeding
        # # ========================================================================
        
        # # ========================================================================
        # # STEP 21: Move UR15 to return tool_extract and then get tool_rotate
        # # ========================================================================
        # print("\n📌 Step 21: Executing UR15 tool exchange operation - return tool_extract and get tool_rotate")
        # print("-" * 40)
        
        # try:
        #     # Execute tool exchange operation using ur_operate_tools
        #     if self.ur_operate_tools:
        #         tool_exchange_result = self.ur_operate_tools.return_tool1_get_tool2_from_task(tool1_name="tool_extract", tool2_name="tool_rotate")
                
        #         if tool_exchange_result:
        #             print("✓ UR15 tool exchange operation (tool_extract → tool_rotate) completed successfully")
        #             step_status["Step 21: UR15 tool exchange (extract→rotate)"] = "SUCCESS"
        #         else:
        #             print("✗ UR15 tool exchange operation (tool_extract → tool_rotate) failed")
        #             step_status["Step 21: UR15 tool exchange (extract→rotate)"] = "FAILED"
        #             self._print_execution_summary(step_status)
        #             return False
        #     else:
        #         print("✗ UR operate tools not initialized")
        #         step_status["Step 21: UR15 tool exchange (extract→rotate)"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool exchange operation: {e}")
        #     step_status["Step 21: UR15 tool exchange (extract→rotate)"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 22: UR15 use FTC to unlock knobs on the server after insertion then push server into the rack
        # # ========================================================================
        # print("\n📌 Step 22: Executing UR15 unlock knob insert task")
        # print("-" * 40)
        
        # try:
        #     # Execute unlock knob insert operation
        #     unlock_knob_insert_result = self.ur15_execute_unlock_knob_insert_task()
            
        #     if unlock_knob_insert_result:
        #         print("✓ UR15 unlock knob insert task completed successfully")
        #         step_status["Step 22: UR15 unlock knob insert"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 unlock knob insert task failed")
        #         step_status["Step 22: UR15 unlock knob insert"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 unlock knob insert task: {e}")
        #     step_status["Step 22: UR15 unlock knob insert"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 23: Move UR15 to return tool_rotate and then get tool_pushpull
        # # ========================================================================
        # print("\n📌 Step 23: Executing UR15 tool exchange operation - return tool_rotate and get tool_pushpull")
        # print("-" * 40)
        
        # try:
        #     # Execute tool exchange operation using ur_operate_tools
        #     if self.ur_operate_tools:
        #         tool_exchange_result2 = self.ur_operate_tools.return_tool1_get_tool2_from_task(tool1_name="tool_rotate", tool2_name="tool_pushpull")
                
        #         if tool_exchange_result2:
        #             print("✓ UR15 tool exchange operation (tool_rotate → tool_pushpull) completed successfully")
        #             step_status["Step 23: UR15 tool exchange (rotate→pushpull)"] = "SUCCESS"
        #         else:
        #             print("✗ UR15 tool exchange operation (tool_rotate → tool_pushpull) failed")
        #             step_status["Step 23: UR15 tool exchange (rotate→pushpull)"] = "FAILED"
        #             self._print_execution_summary(step_status)
        #             return False
        #     else:
        #         print("✗ UR operate tools not initialized")
        #         step_status["Step 23: UR15 tool exchange (rotate→pushpull)"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool exchange operation: {e}")
        #     step_status["Step 23: UR15 tool exchange (rotate→pushpull)"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 24: UR15 use FTC to close handles on the server after insertion
        # # ========================================================================
        # print("\n📌 Step 24: Executing UR15 close handles task")
        # print("-" * 40)
        
        # try:
        #     # Execute close handles operation
        #     close_handles_result = self.ur15_execute_close_handles_task()
            
        #     if close_handles_result:
        #         print("✓ UR15 close handles task completed successfully")
        #         step_status["Step 24: UR15 close handles"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 close handles task failed")
        #         step_status["Step 24: UR15 close handles"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 close handles task: {e}")
        #     step_status["Step 24: UR15 close handles"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 25: Move UR15 to return tool_pushpull
        # # ========================================================================
        # print("\n📌 Step 25: Executing UR15 return tool_pushpull to storage position")
        # print("-" * 40)
        
        # try:
        #     # Execute tool return operation using ur_operate_tools
        #     if self.ur_operate_tools:
        #         tool_return_result = self.ur_operate_tools.return_tool_from_task_position("tool_pushpull")
                
        #         if tool_return_result:
        #             print("✓ UR15 tool_pushpull return to storage completed successfully")
        #             step_status["Step 25: UR15 return tool_pushpull"] = "SUCCESS"
        #         else:
        #             print("✗ UR15 tool_pushpull return to storage failed")
        #             step_status["Step 25: UR15 return tool_pushpull"] = "FAILED"
        #             self._print_execution_summary(step_status)
        #             return False
        #     else:
        #         print("✗ UR operate tools not initialized")
        #         step_status["Step 25: UR15 return tool_pushpull"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 tool return operation: {e}")
        #     step_status["Step 25: UR15 return tool_pushpull"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False

        # # ========================================================================
        # # STEP 26: Move UR15 to home position using movej in reverse joint order
        # # ========================================================================
        # print("\n📌 Step 26: Executing UR15 movej to home position (J6 -> J1)")
        # print("-" * 40)
        
        # try:
        #     # Execute movej to home position operation
        #     movej_result = self.ur15_execute_movej_to_home_position()
            
        #     if movej_result:
        #         print("✓ UR15 movej to home position completed successfully")
        #         step_status["Step 26: UR15 move to target position"] = "SUCCESS"
        #     else:
        #         print("✗ UR15 movej to home position failed")
        #         step_status["Step 26: UR15 move to target position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during UR15 movej to home position: {e}")
        #     step_status["Step 26: UR15 move to target position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 27: AMR move arm from side position to dock position
        # # ========================================================================
        # print("\n📌 Step 27: Executing AMR arm movement from side to dock")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR arm movement from side to dock
        #     arm_return_result = self.amr_controller.amr_move_arm_from_side_to_dock()
            
        #     if arm_return_result and arm_return_result.get('success', False):
        #         print("✓ AMR arm movement from side to dock completed successfully")
        #         step_status["Step 27: AMR arm from side to dock"] = "SUCCESS"
        #     else:
        #         print("✗ AMR arm movement from side to dock failed")
        #         step_status["Step 27: AMR arm from side to dock"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR arm movement: {e}")
        #     step_status["Step 27: AMR arm from side to dock"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # # ========================================================================
        # # STEP 28: AMR back to home position
        # # ========================================================================
        # print("\n📌 Step 28: Moving AMR back to home position")
        # print("-" * 40)
        
        # try:
        #     # Execute AMR movement back to home position (LM2)
        #     print("   → Moving to LM2 (home position)...")
        #     home_result = self.amr_controller.amr_controller.goto(target_id="LM2", wait=True)
            
        #     if home_result.get('success', False):
        #         print("✓ AMR successfully returned to home position (LM2)")
        #         step_status["Step 28: AMR back to home position"] = "SUCCESS"
        #     else:
        #         print("✗ AMR failed to return to home position")
        #         step_status["Step 28: AMR back to home position"] = "FAILED"
        #         self._print_execution_summary(step_status)
        #         return False
                
        # except Exception as e:
        #     print(f"✗ Error during AMR return to home: {e}")
        #     step_status["Step 28: AMR back to home position"] = "FAILED"
        #     self._print_execution_summary(step_status)
        #     return False
        
        # Print execution summary
        self._print_execution_summary(step_status)



if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Demo Task Manager for robot coordination')
    parser.add_argument('--server-index', type=int, default=15,
                        help='Server index for identification in redis (optional)')
    args = parser.parse_args()
    
    # Create task manager instance with server index
    task_manager = TaskManager(server_index=args.server_index)
    print("Task Manager initialization completed")
    
    # Test the complete sequence execution
    print("\nTesting complete task sequence...")
    success = task_manager.execute_complete_task_sequence()
    
    if success:
        print("\n🎉 Complete task sequence finished successfully!")
    else:
        print("\n❌ Complete task sequence failed!")
    