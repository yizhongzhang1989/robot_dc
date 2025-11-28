#!/usr/bin/env python3
"""
UR15 Workflow Runner - Main script to execute robot operations using workflow engine

This script demonstrates the modular architecture where:
1. WorkflowEngine controls the overall process
2. OperationHandlers perform specific tasks (move, capture, position, etc.)
3. Workflow configuration defines what to do and in what order
4. Shared context allows data flow between operations

Usage:
    python3 -m ur15_workflow.runner --config ur15_workflow/config/workflow_example.yaml
    python3 -m ur15_workflow.runner --config workflow.yaml --dry-run
"""

import sys
import os
from pathlib import Path

# Add package root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node

# Import workflow components
from ur15_workflow.engine import WorkflowEngine
from ur15_workflow.handlers import register_all_handlers

# Import positioning and status utilities
# Try to find scripts directory
scripts_dir = None
# 1. Try absolute path (dev env)
if os.path.exists("/home/a/Documents/robot_dc/scripts"):
    scripts_dir = Path("/home/a/Documents/robot_dc/scripts")
# 2. Try relative to source (if running from source)
elif (Path(__file__).parents[4] / 'scripts').exists():
    scripts_dir = Path(__file__).parents[4] / 'scripts'

if scripts_dir:
    # Positioning3DWebAPIClient
    try:
        sys.path.append(str(scripts_dir / 'ThirdParty' / 'robot_vision'))
        from core.positioning_3d_webapi import Positioning3DWebAPIClient
    except ImportError:
        print("Warning: Positioning3DWebAPI not available (ImportError)")
        Positioning3DWebAPIClient = None

    # RobotStatusClient (use Redis-based version)
    try:
        from robot_status_redis.client_utils import RobotStatusClient
    except ImportError:
        print("Warning: RobotStatusClient not available (ImportError)")
        RobotStatusClient = None
else:
    print("Warning: Could not find 'scripts' directory. External services will be unavailable.")
    Positioning3DWebAPIClient = None
    RobotStatusClient = None


class RobotWorkflowRunner:
    """
    Main runner that sets up environment and executes workflows
    """
    
    def __init__(self, workflow_config_path: str):
        """
        Initialize workflow runner
        
        Args:
            workflow_config_path: Path to workflow configuration file
        """
        self.workflow_config_path = workflow_config_path
        self.workflow_engine = None
        self.rclpy_initialized = False
        
    def setup(self) -> bool:
        """
        Setup environment and initialize components
        
        Returns:
            True if setup successful
        """
        print("\n" + "="*70)
        print(" UR15 Workflow Runner - Setup")
        print("="*70)
        
        try:
            # Initialize ROS2
            print("\n>>> Initializing ROS2...")
            rclpy.init()
            self.rclpy_initialized = True
            print("✓ ROS2 initialized")
            
            # Create workflow engine
            print("\n>>> Creating workflow engine...")
            self.workflow_engine = WorkflowEngine(config_path=self.workflow_config_path)
            
            if not self.workflow_engine.workflow:
                print("✗ No workflow loaded")
                return False
            
            print(f"✓ Workflow loaded with {len(self.workflow_engine.workflow)} operations")
            
            # Register all operation handlers
            print("\n>>> Registering operation handlers...")
            register_all_handlers(self.workflow_engine)
            
            # Initialize services in context
            print("\n>>> Initializing services...")
            self._initialize_services()
            
            print("\n" + "="*70)
            print(" Setup Complete")
            print("="*70)
            
            return True
            
        except Exception as e:
            print(f"\n✗ Setup failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _initialize_services(self):
        """
        Initialize external services (positioning, robot_status, etc.)
        """
        context = self.workflow_engine.context
        
        # Initialize Positioning Service
        if Positioning3DWebAPIClient is not None:
            positioning_service_url = context.get('positioning_service_url', 'http://localhost:8004')
            try:
                positioning_client = Positioning3DWebAPIClient(service_url=positioning_service_url)
                health = positioning_client.check_health()
                
                if health.get('success'):
                    context['positioning_client'] = positioning_client
                    print(f"✓ Positioning service connected: {positioning_service_url}")
                else:
                    print(f"✗ Positioning service not healthy: {health.get('error')}")
                    
            except Exception as e:
                print(f"✗ Failed to connect to positioning service: {e}")
        else:
            print("⊘ Positioning service not available (module not imported)")
        
        # Initialize Robot Status Service (Redis-based)
        if RobotStatusClient is not None:
            try:
                # RobotStatusClient (Redis version) optionally takes a ROS node for logging
                # Create a dummy node for logging purposes
                class DummyNode(Node):
                    def __init__(self):
                        super().__init__('workflow_runner_node')
                
                dummy_node = DummyNode()
                # Redis-based client doesn't need timeout_sec parameter
                robot_status_client = RobotStatusClient(node=dummy_node)
                context['robot_status_client'] = robot_status_client
                context['ros_node'] = dummy_node
                print("✓ RobotStatusClient initialized (Redis backend)")
                
            except Exception as e:
                print(f"✗ Failed to initialize RobotStatusClient: {e}")
        else:
            print("⊘ RobotStatusClient not available (module not imported)")
    
    def execute(self) -> dict:
        """
        Execute the workflow
        
        Returns:
            Dictionary of results from all operations
        """
        if self.workflow_engine is None:
            print("✗ Workflow engine not initialized")
            return {}
        
        # Execute workflow
        results = self.workflow_engine.execute()
        
        return results
    
    def cleanup(self):
        """
        Cleanup resources
        """
        print("\n>>> Cleaning up resources...")
        
        # Disconnect robot
        if self.workflow_engine and self.workflow_engine.context.get('robot'):
            try:
                robot = self.workflow_engine.context['robot']
                robot.close()
                print("✓ Robot disconnected")
            except Exception as e:
                print(f"✗ Error disconnecting robot: {e}")
        
        # Shutdown ROS2
        if self.rclpy_initialized:
            try:
                rclpy.shutdown()
                print("✓ ROS2 shutdown")
            except Exception as e:
                print(f"✗ Error shutting down ROS2: {e}")
    
    def run(self) -> int:
        """
        Full workflow execution with setup and cleanup
        
        Returns:
            Exit code (0 = success, non-zero = error)
        """
        try:
            # Setup
            if not self.setup():
                return 1
            
            # Execute workflow
            results = self.execute()
            
            # Save results
            output_path = os.path.join(
                os.path.dirname(self.workflow_config_path),
                'workflow_results.json'
            )
            self.workflow_engine.save_results(output_path)
            
            # Check if all operations succeeded
            all_success = all(
                r.get('status') == 'success' 
                for r in results.values()
            )
            
            return 0 if all_success else 1
            
        except KeyboardInterrupt:
            print("\n\n🛑 Workflow interrupted by user")
            return 130
            
        except Exception as e:
            print(f"\n✗ Fatal error: {e}")
            import traceback
            traceback.print_exc()
            return 1
            
        finally:
            self.cleanup()


def main():
    """
    Main entry point
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description='UR15 Workflow Runner - Execute robot operations from workflow configuration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with example workflow
  python3 -m ur15_workflow.runner --config ur15_workflow/config/workflow_example.yaml
  
  # Run with simple workflow
  python3 -m ur15_workflow.runner --config ur15_workflow/config/workflow_simple.yaml
  
  # Dry run (validate workflow without execution)
  python3 -m ur15_workflow.runner --config workflow.yaml --dry-run
        """
    )
    
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help='Path to workflow configuration file (YAML or JSON)'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate workflow configuration without executing'
    )
    
    args = parser.parse_args()
    
    # Validate config file exists
    if not os.path.exists(args.config):
        print(f"✗ Configuration file not found: {args.config}")
        return 1
    
    if args.dry_run:
        print("\n>>> Dry run mode - validating workflow configuration...")
        engine = WorkflowEngine(config_path=args.config)
        if engine.workflow:
            print("\n✓ Workflow configuration valid")
            print(f"  Operations: {len(engine.workflow)}")
            for idx, op in enumerate(engine.workflow, 1):
                op_id = op.get('id', 'unknown')
                op_type = op.get('type', 'unknown')
                print(f"    {idx}. {op_id} ({op_type})")
            return 0
        else:
            print("\n✗ Invalid workflow configuration")
            return 1
    
    # Run workflow
    runner = RobotWorkflowRunner(workflow_config_path=args.config)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
