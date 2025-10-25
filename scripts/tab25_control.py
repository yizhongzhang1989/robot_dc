#!/usr/bin/env python3
"""
TAB25 Control Script - Sequential Task Execution

This script provides a series of blocking functions to control the robot
through the web API. Functions are executed sequentially by the run() function.

Author: Assistant
Date: October 25, 2025
"""

import sys
import os

# Add the ThirdParty directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
third_party_dir = os.path.join(script_dir, 'ThirdParty')
sys.path.insert(0, third_party_dir)

from seer_control.dc_demo_2025_webapi_controller import DCDemo2025WebAPIController

# Global controller instance
amr_controller = None

# Web API URL
AMR_WEB_API_URL = "http://192.168.1.142:5000"


def init():
    """
    Initialize the global DCDemo2025WebAPIController with the web API URL.
    """
    global amr_controller
    print("="*60)
    print("🤖 TAB25 Control Script - Initializing")
    print("="*60)
    print(f"🌐 Web API URL: {AMR_WEB_API_URL}")
    
    amr_controller = DCDemo2025WebAPIController(AMR_WEB_API_URL)
    
    # Check connection status
    if amr_controller.is_connected():
        print("✅ Robot is connected")
    else:
        print("⚠️  Robot is not connected")
        print("   Please ensure the web server is running and robot is connected")
    
    print("-"*60)


def amr_home():
    """
    Function 1: Move AMR to home position (LM2).
    
    Calls goto(target_id="LM2") with wait=True (blocking).
    """
    print("\n📍 Function 1: AMR Home")
    print("   Moving to home position LM2...")
    
    result = amr_controller.goto(target_id="LM2", wait=True)
    
    if result.get('success', False):
        print(f"✅ Successfully reached home position")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to reach home position: {result.get('message', 'Unknown error')}")
    
    return result


def amr_smalltest():
    """
    Function 2: Execute small test trajectory.
    
    Calls navigate(trajectory="smalltest") with wait=True (blocking).
    """
    print("\n🚀 Function 2: AMR Small Test")
    print("   Executing 'smalltest' trajectory...")
    
    result = amr_controller.navigate(trajectory="smalltest", wait=True)
    
    if result.get('success', False):
        print(f"✅ Successfully completed smalltest trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete smalltest trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def run():
    """
    Main execution function.
    
    Calls all functions in sequence:
    1. amr_home() - Move to home position
    2. amr_smalltest() - Execute small test trajectory
    """
    print("\n" + "="*60)
    print("🏃 Starting Sequential Task Execution")
    print("="*60)
    
    try:
        # Function 1: Move to home
        amr_home()
        
        # Function 2: Execute small test
        amr_smalltest()
        
        print("\n" + "="*60)
        print("✅ All tasks completed successfully")
        print("="*60)
    
    except Exception as e:
        print(f"\n❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        print("="*60)


def main():
    """
    Main entry point.
    """
    # Initialize controller
    init()
    
    # Execute sequential tasks
    run()


if __name__ == "__main__":
    main()
