#!/usr/bin/env python3
"""
AMR Control Script - Autonomous Mobile Robot Controller
"""

import sys
import os
import yaml
import time

# Add workspace paths
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root, get_scripts_directory
    repo_root_path = get_workspace_root()
    script_dir = get_scripts_directory()
except ImportError:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root_path = os.path.abspath(os.path.join(script_dir, '..'))

# Add ThirdParty directory to path
third_party_dir = os.path.join(script_dir, 'ThirdParty')
sys.path.insert(0, third_party_dir)

# Import AMR controller
from seer_control.dc_demo_2025_webapi_controller import DCDemo2025WebAPIController


class AMRController:
    """
    AMR (Autonomous Mobile Robot) Controller Class
    
    This class provides a high-level interface for controlling AMR navigation,
    loading configuration from robot_config.yaml file.
    """
    
    def __init__(self):
        """
        Initialize the AMR Controller.
        """
        self.amr_controller = None
        self.amr_web_api_url = None
        self.config = None
        
        # Load configuration and initialize controller
        self._load_amr_config_from_files()
        self._initialize_amr()
    
    def _load_amr_config_from_files(self):
        """
        Private method to load AMR configuration from robot_config.yaml.
        """
        config_path = os.path.join(repo_root_path, 'config', 'robot_config.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            
            # Extract AMR configuration
            amr_config = self.config.get('amr', {})
            # Use AMR IP as the host for connecting to the web API
            amr_web_host = amr_config.get('ip', 'msra-yizhong.guest.corp.microsoft.com')
            web_config = amr_config.get('web', {})
            web_port = web_config.get('port', 5000)
            
            # Construct AMR Web API URL
            self.amr_web_api_url = f"http://{amr_web_host}:{web_port}"
            
            print(f"📋 Loaded AMR configuration from: {config_path}")
            print(f"🌐 AMR Web API URL: {self.amr_web_api_url}")
            
        except FileNotFoundError:
            print(f"⚠️  Configuration file not found: {config_path}")
            print("   Using default AMR Web API URL: http://msra-yizhong.guest.corp.microsoft.com:5000")
            self.amr_web_api_url = "http://msra-yizhong.guest.corp.microsoft.com:5000"
            
        except yaml.YAMLError as e:
            print(f"❌ Error parsing YAML configuration: {e}")
            print("   Using default AMR Web API URL: http://msra-yizhong.guest.corp.microsoft.com:5000")
            self.amr_web_api_url = "http://msra-yizhong.guest.corp.microsoft.com:5000"
            
        except Exception as e:
            print(f"❌ Unexpected error loading configuration: {e}")
            print("   Using default AMR Web API URL: http://msra-yizhong.guest.corp.microsoft.com:5000")
            self.amr_web_api_url = "http://msra-yizhong.guest.corp.microsoft.com:5000"
    
    def _initialize_amr(self):
        """
        Private method to initialize the AMR controller.
        """
        # Initialize AMR controller
        self.amr_controller = DCDemo2025WebAPIController(self.amr_web_api_url)
        
        # Check AMR connection status
        if self.amr_controller.is_connected():
            print("✅ AMR is connected")
        else:
            print("⚠️  AMR is not connected")
            print("   Please ensure the web server is running and robot is connected")
        
        print("-"*60)
    
    def is_connected(self):
        """
        Check if AMR is connected.
        
        Returns:
            bool: True if connected, False otherwise
        """
        return self.amr_controller.is_connected() if self.amr_controller else False

# ================================ Methods for Basic AMR Movements =========================================
    def amr_move_to_home_position(self):
        """
        Move AMR to home position (LM2).
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📍 AMR Home")
        print("   Moving to home position LM2...")
        
        result = self.amr_controller.goto(target_id="LM2", wait=True)
        
        if result.get('success', False):
            print(f"✅ Successfully reached home position")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to reach home position: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_execute_small_test_trajectory(self):
        """
        Execute small test trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n🚀 AMR Small Test")
        print("   Executing 'smalltest' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="smalltest", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed smalltest trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete smalltest trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_execute_loop_test_trajectory(self):
        """
        Execute loop test trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n🔄 AMR Loop Test")
        print("   Executing 'looptest' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="looptest", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed looptest trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete looptest trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_to_target_position(self, target_id, wait=True):
        """
        Move AMR to a specific landmark or position.
        """
        print(f"\n📍 AMR Move to Target: {target_id}")
        print(f"   Moving to target: {target_id}...")
        
        result = self.amr_controller.goto(target_id=target_id, wait=wait)
        
        if result.get('success', False):
            print(f"✅ Successfully reached target: {target_id}")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to reach target {target_id}: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_navigate_bytrajectory(self, trajectory, wait=True):
        """
        Execute a custom trajectory.
        """
        print(f"\n🚀 AMR Navigate: {trajectory}")
        print(f"   Executing trajectory: {trajectory}...")
        
        result = self.amr_controller.navigate(trajectory=trajectory, wait=wait)
        
        if result.get('success', False):
            print(f"✅ Successfully completed trajectory: {trajectory}")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete trajectory {trajectory}: {result.get('message', 'Unknown error')}")
        
        return result

# ================================ Methods for Arm Trajectories =========================================
    def amr_move_arm_from_dock_to_rack(self):
        """
        Execute arm dock to rack trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📦 AMR Arm Dock to Rack")
        print("   Executing 'arm_dock2rack' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_dock2rack", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_dock2rack trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_dock2rack trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_arm_from_rack_to_side(self):
        """
        Execute arm rack to side trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n🔄 AMR Arm Rack to Side")
        print("   Executing 'arm_rack2side' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_rack2side", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_rack2side trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_rack2side trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_arm_from_side_to_rack(self):
        """
        Execute arm side to rack trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📦 AMR Arm Side to Rack")
        print("   Executing 'arm_side2rack' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_side2rack", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_side2rack trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_side2rack trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_arm_from_rack_to_dock(self):
        """
        Execute arm rack to dock trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📦 AMR Arm Rack to Dock")
        print("   Executing 'arm_rack2dock' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_rack2dock", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_rack2dock trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_rack2dock trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_arm_from_dock_to_side(self):
        """
        Execute arm dock to side trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📦 AMR Arm Dock to Side")
        print("   Executing 'arm_dock2side' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_dock2side", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_dock2side trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_dock2side trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_arm_from_side_to_dock(self):
        """
        Execute arm side to dock trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📦 AMR Arm Side to Dock")
        print("   Executing 'arm_side2dock' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="arm_side2dock", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed arm_side2dock trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete arm_side2dock trajectory: {result.get('message', 'Unknown error')}")
        
        return result

# ================================= Methods for Courier Robot Trajectories =========================================  
    def amr_move_courier_from_dock_to_extraction_position(self):
        """
        Execute courier dock1 to rack trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📮 AMR Courier Dock to Extraction Position")
        print("   Executing 'courier_dock2rack1' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="courier_dock2rack1", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed courier_dock2rack1 trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete courier_dock2rack1 trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_courier_from_dock_to_insertion_position(self):
        """
        Execute courier dock2 to rack trajectory.
        
        Returns:
            dict: Result dictionary with success status and optional task_id
        """
        print("\n📮 AMR Courier Dock to Insertion Position")
        print("   Executing 'courier_dock2rack2' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="courier_dock2rack2", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed courier_dock2rack2 trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete courier_dock2rack2 trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_courier_from_extraction_position_to_dock(self):
        """
        Execute courier rack to dock1 trajectory (position for extraction).
        """
        print("\n📮 AMR Courier Extraction Position to Dock")
        print("   Executing 'courier_rack2dock1' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="courier_rack2dock1", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed courier_rack2dock1 trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete courier_rack2dock1 trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
    def amr_move_courier_from_insertion_position_to_dock(self):
        """
        Execute courier rack to dock2 trajectory (position for insertion).
        """
        print("\n📮 AMR Courier Insertion Position to Dock")
        print("   Executing 'courier_rack2dock2' trajectory...")
        
        result = self.amr_controller.navigate(trajectory="courier_rack2dock2", wait=True)
        
        if result.get('success', False):
            print("✅ Successfully completed courier_rack2dock2 trajectory")
            if 'task_id' in result:
                print(f"   Task ID: {result['task_id']}")
        else:
            print(f"❌ Failed to complete courier_rack2dock2 trajectory: {result.get('message', 'Unknown error')}")
        
        return result
    
def main():
    """
    Main entry point for the AMR control script.
    """
    # Initialize AMR controller
    amr = AMRController()


if __name__ == "__main__":
    main()