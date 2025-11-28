#!/usr/bin/env python3
"""
Workflow Engine - Controls the entire robot operation process based on workflow configuration
"""

import json
import yaml
import time
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime


class WorkflowEngine:
    """
    Central controller that executes a list of operations in sequence.
    Each operation is defined by its type, parameters, and actions.
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize workflow engine
        
        Args:
            config_path: Path to workflow configuration file (YAML or JSON)
        """
        self.workflow: List[Dict[str, Any]] = []
        self.context: Dict[str, Any] = {}  # Shared context between operations
        self.results: Dict[str, Any] = {}  # Store results from each operation
        self.operation_handlers = {}  # Registry of operation type handlers
        
        if config_path:
            self.load_workflow(config_path)
    
    def load_workflow(self, config_path: str) -> bool:
        """
        Load workflow from YAML or JSON file
        
        Args:
            config_path: Path to configuration file
            
        Returns:
            True if loaded successfully
        """
        try:
            config_file = Path(config_path)
            
            if not config_file.exists():
                print(f"✗ Workflow config not found: {config_path}")
                return False
            
            with open(config_file, 'r') as f:
                if config_file.suffix in ['.yaml', '.yml']:
                    config = yaml.safe_load(f)
                elif config_file.suffix == '.json':
                    config = json.load(f)
                else:
                    print(f"✗ Unsupported config format: {config_file.suffix}")
                    return False
            
            # Extract workflow operations
            self.workflow = config.get('workflow', [])
            
            # Extract global context if present
            if 'context' in config:
                self.context.update(config['context'])
            
            print(f"✓ Loaded workflow with {len(self.workflow)} operations")
            return True
            
        except Exception as e:
            print(f"✗ Error loading workflow: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def set_workflow(self, workflow: List[Dict[str, Any]]):
        """
        Set workflow programmatically
        
        Args:
            workflow: List of operation definitions
        """
        self.workflow = workflow
        print(f"✓ Workflow set with {len(self.workflow)} operations")
    
    def register_handler(self, operation_type: str, handler_class):
        """
        Register a handler class for a specific operation type
        
        Args:
            operation_type: Type identifier (e.g., "capture_positioning")
            handler_class: Class that implements execute(operation, context, results)
        """
        self.operation_handlers[operation_type] = handler_class
        print(f"✓ Registered handler for '{operation_type}'")
    
    def execute(self) -> Dict[str, Any]:
        """
        Execute the entire workflow in sequence
        
        Returns:
            Dictionary containing results from all operations
        """
        print("\n" + "="*70)
        print(" Starting Workflow Execution")
        print("="*70)
        print(f"Total operations: {len(self.workflow)}")
        print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)
        
        start_time = time.time()
        
        for idx, operation in enumerate(self.workflow, 1):
            operation_id = operation.get('id', f'operation_{idx}')
            operation_type = operation.get('type', 'unknown')
            
            print(f"\n>>> [{idx}/{len(self.workflow)}] Executing: {operation_id}")
            print(f"    Type: {operation_type}")
            
            try:
                # Check if this operation should be skipped based on conditions
                if not self._check_conditions(operation):
                    print("    ⊘ Skipped (conditions not met)")
                    self.results[operation_id] = {
                        'status': 'skipped',
                        'reason': 'conditions_not_met'
                    }
                    continue
                
                # Execute the operation
                op_start_time = time.time()
                result = self._execute_operation(operation)
                op_elapsed = time.time() - op_start_time
                
                # Store result
                result['elapsed_time'] = op_elapsed
                self.results[operation_id] = result
                
                # Check if operation succeeded
                if result.get('status') == 'success':
                    print(f"    ✓ Completed successfully ({op_elapsed:.2f}s)")
                else:
                    print(f"    ✗ Failed: {result.get('error', 'Unknown error')}")
                    
                    # Check if we should stop on failure
                    if operation.get('stop_on_failure', False):
                        print(f"\n✗ Workflow stopped due to failure in '{operation_id}'")
                        break
                
            except KeyboardInterrupt:
                print(f"\n\n🛑 Workflow interrupted by user at '{operation_id}'")
                self.results[operation_id] = {
                    'status': 'interrupted',
                    'error': 'User interrupted'
                }
                break
                
            except Exception as e:
                print(f"    ✗ Exception: {e}")
                import traceback
                traceback.print_exc()
                
                self.results[operation_id] = {
                    'status': 'error',
                    'error': str(e)
                }
                
                if operation.get('stop_on_failure', False):
                    print(f"\n✗ Workflow stopped due to exception in '{operation_id}'")
                    break
        
        # Print summary
        total_elapsed = time.time() - start_time
        self._print_summary(total_elapsed)
        
        return self.results
    
    def _execute_operation(self, operation: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a single operation using its registered handler
        
        Args:
            operation: Operation definition
            
        Returns:
            Result dictionary
        """
        operation_type = operation.get('type', 'unknown')
        
        # Check if handler is registered
        if operation_type not in self.operation_handlers:
            return {
                'status': 'error',
                'error': f"No handler registered for type '{operation_type}'"
            }
        
        # Get handler class and instantiate
        handler_class = self.operation_handlers[operation_type]
        handler = handler_class()
        
        # Execute operation
        result = handler.execute(operation, self.context, self.results)
        
        # Update context with any outputs from this operation
        if 'outputs' in result:
            self.context.update(result['outputs'])
        
        return result
    
    def _check_conditions(self, operation: Dict[str, Any]) -> bool:
        """
        Check if operation conditions are met
        
        Args:
            operation: Operation definition
            
        Returns:
            True if conditions are met (or no conditions specified)
        """
        conditions = operation.get('conditions', [])
        
        if not conditions:
            return True  # No conditions means always execute
        
        for condition in conditions:
            condition_type = condition.get('type')
            
            if condition_type == 'previous_success':
                # Check if a previous operation succeeded
                required_op = condition.get('operation_id')
                if required_op not in self.results:
                    return False
                if self.results[required_op].get('status') != 'success':
                    return False
            
            elif condition_type == 'context_value':
                # Check if context has a specific value
                key = condition.get('key')
                expected_value = condition.get('value')
                if self.context.get(key) != expected_value:
                    return False
            
            # Add more condition types as needed
        
        return True
    
    def _print_summary(self, total_elapsed: float):
        """
        Print execution summary
        
        Args:
            total_elapsed: Total execution time in seconds
        """
        print("\n" + "="*70)
        print(" Workflow Execution Summary")
        print("="*70)
        
        total_ops = len(self.workflow)
        success_count = sum(1 for r in self.results.values() if r.get('status') == 'success')
        failed_count = sum(1 for r in self.results.values() if r.get('status') == 'error')
        skipped_count = sum(1 for r in self.results.values() if r.get('status') == 'skipped')
        
        print(f"Total operations: {total_ops}")
        print(f"  Successful: {success_count}")
        print(f"  Failed: {failed_count}")
        print(f"  Skipped: {skipped_count}")
        print(f"Total time: {total_elapsed:.2f}s")
        print()
        
        for operation in self.workflow:
            operation_id = operation.get('id', 'unknown')
            if operation_id in self.results:
                result = self.results[operation_id]
                status = result.get('status', 'unknown')
                elapsed = result.get('elapsed_time', 0)
                
                status_symbol = {
                    'success': '✓',
                    'error': '✗',
                    'skipped': '⊘',
                    'interrupted': '🛑'
                }.get(status, '?')
                
                print(f"  {status_symbol} {operation_id}: {status.upper()} ({elapsed:.2f}s)")
                
                if 'error' in result:
                    print(f"      Error: {result['error']}")
            else:
                print(f"  ⊘ {operation_id}: NOT EXECUTED")
        
        print("="*70)
    
    def save_results(self, output_path: str):
        """
        Save workflow results to JSON file
        
        Args:
            output_path: Path to save results
        """
        try:
            output_data = {
                'timestamp': datetime.now().isoformat(),
                'workflow': self.workflow,
                'results': self.results,
                'context': self.context
            }
            
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            
            print(f"\n💾 Workflow results saved to: {output_path}")
            
        except Exception as e:
            print(f"\n✗ Failed to save results: {e}")
