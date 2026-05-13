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

            # Disambiguate when the same id is reused (e.g. multiple
            # `movej_to_pose` steps in a row). Without this, repeated keys
            # silently overwrite earlier entries in self.results, so the
            # final log and the summary report only the LAST occurrence as
            # if the earlier ones never ran.
            result_key = operation_id
            if result_key in self.results:
                suffix = 2
                while f"{operation_id}#{suffix}" in self.results:
                    suffix += 1
                result_key = f"{operation_id}#{suffix}"

            print(f"\n>>> [{idx}/{len(self.workflow)}] Executing: {result_key}")
            print(f"    Type: {operation_type}")
            
            try:
                # Check if this operation should be skipped based on conditions
                if not self._check_conditions(operation):
                    print("    ⊘ Skipped (conditions not met)")
                    self.results[result_key] = {
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
                self.results[result_key] = result
                
                # Check if operation succeeded
                if result.get('status') == 'success':
                    print(f"    ✓ Completed successfully ({op_elapsed:.2f}s)")
                else:
                    print(f"    ✗ Failed: {result.get('error', 'Unknown error')}")
                    
                    # Check if we should stop on failure
                    if operation.get('stop_on_failure', False):
                        print(f"\n✗ Workflow stopped due to failure in '{result_key}'")
                        break
                
            except KeyboardInterrupt:
                print(f"\n\n🛑 Workflow interrupted by user at '{result_key}'")
                self.results[result_key] = {
                    'status': 'interrupted',
                    'error': 'User interrupted'
                }
                break
                
            except Exception as e:
                print(f"    ✗ Exception: {e}")
                import traceback
                traceback.print_exc()
                
                self.results[result_key] = {
                    'status': 'error',
                    'error': str(e)
                }
                
                if operation.get('stop_on_failure', False):
                    print(f"\n✗ Workflow stopped due to exception in '{result_key}'")
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

        # Walk the workflow in order, regenerating the same disambiguation
        # logic as execute(), so each step shows its own result entry even
        # when the same operation id repeats.
        seen_ids: Dict[str, int] = {}
        for idx, operation in enumerate(self.workflow, 1):
            operation_id = operation.get('id', f'operation_{idx}')
            seen_ids[operation_id] = seen_ids.get(operation_id, 0) + 1
            if seen_ids[operation_id] == 1:
                key = operation_id
            else:
                key = f"{operation_id}#{seen_ids[operation_id]}"

            if key in self.results:
                result = self.results[key]
                status = result.get('status', 'unknown')
                elapsed = result.get('elapsed_time', 0)
                
                status_symbol = {
                    'success': '✓',
                    'error': '✗',
                    'skipped': '⊘',
                    'interrupted': '🛑'
                }.get(status, '?')
                
                print(f"  {status_symbol} {key}: {status.upper()} ({elapsed:.2f}s)")
                
                if 'error' in result:
                    print(f"      Error: {result['error']}")
            else:
                print(f"  ⊘ {key}: NOT EXECUTED")
        
        print("="*70)
    
    def save_results(self, output_path: str, save_images: bool = False):
        """
        Save workflow results to JSON file as a clean log report
        
        Args:
            output_path: Path to save results
            save_images: If True, save images separately and include paths; if False (default), exclude image data
        """
        try:
            import numpy as np
            import cv2
            
            # Helper function to convert non-serializable objects and filter large data
            def make_serializable(obj, path="", is_output=False):
                """
                Convert objects to JSON-serializable format, filtering out large image data
                
                Args:
                    obj: Object to convert
                    path: Current path in the object tree (for debugging)
                    is_output: Whether this is inside an 'outputs' dict
                """
                if isinstance(obj, np.ndarray):
                    # Check if this is likely image data (large array with image-like shape)
                    if len(obj.shape) >= 2 and obj.size > 1000:
                        # This is likely an image - return metadata instead
                        return {
                            "_type": "image_data",
                            "shape": list(obj.shape),
                            "dtype": str(obj.dtype),
                            "size_mb": round(obj.nbytes / (1024 * 1024), 2),
                            "_note": "Image data excluded from log. Set save_images=True to save separately."
                        }
                    else:
                        # Small arrays (like coordinates) can be included
                        return obj.tolist()
                elif isinstance(obj, dict):
                    result = {}
                    for k, v in obj.items():
                        # Skip image data keys in outputs
                        if is_output and isinstance(v, np.ndarray) and len(v.shape) >= 2 and v.size > 1000:
                            result[k] = {
                                "_type": "image_data",
                                "shape": list(v.shape),
                                "dtype": str(v.dtype),
                                "size_mb": round(v.nbytes / (1024 * 1024), 2),
                                "_note": "Image data excluded from log"
                            }
                        else:
                            result[k] = make_serializable(v, f"{path}.{k}", k == "outputs")
                    return result
                elif isinstance(obj, list):
                    return [make_serializable(item, f"{path}[{i}]", is_output) for i, item in enumerate(obj)]
                elif isinstance(obj, (str, int, float, bool, type(None))):
                    return obj
                else:
                    # Handle other types
                    return f"<{type(obj).__name__}>"
            
            # Filter context to remove non-serializable and large objects
            serializable_context = {}
            for k, v in self.context.items():
                # Skip image data in context (typically ends with .jpg, .png, etc.)
                if isinstance(v, np.ndarray) and len(v.shape) >= 2 and v.size > 1000:
                    serializable_context[k] = {
                        "_type": "image_data",
                        "shape": list(v.shape),
                        "dtype": str(v.dtype),
                        "_note": "Image data excluded from context"
                    }
                else:
                    serializable_context[k] = make_serializable(v, f"context.{k}")

            output_data = {
                'timestamp': datetime.now().isoformat(),
                'workflow': self.workflow,
                'results': make_serializable(self.results),
                'context': serializable_context,
                '_metadata': {
                    'total_operations': len(self.workflow),
                    'successful': sum(1 for r in self.results.values() if r.get('status') == 'success'),
                    'failed': sum(1 for r in self.results.values() if r.get('status') == 'error'),
                    'skipped': sum(1 for r in self.results.values() if r.get('status') == 'skipped'),
                    'note': 'This is a workflow execution log. Large image data has been excluded to keep file size manageable.'
                }
            }
            
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            
            # Get file size
            file_size = Path(output_path).stat().st_size
            if file_size < 1024:
                size_str = f"{file_size} B"
            elif file_size < 1024 * 1024:
                size_str = f"{file_size / 1024:.2f} KB"
            else:
                size_str = f"{file_size / (1024 * 1024):.2f} MB"
            
            print(f"\n💾 Workflow results saved to: {output_path} ({size_str})")
            
            # Optionally save images separately
            if save_images:
                self._save_images_separately(output_path)
            
        except Exception as e:
            print(f"\n✗ Failed to save results: {e}")
            import traceback
            traceback.print_exc()
    
    def _save_images_separately(self, output_path: str):
        """
        Save captured images to separate files
        
        Args:
            output_path: Base path for the results file
        """
        try:
            import numpy as np
            import cv2
            
            # Create images directory next to results file
            output_dir = Path(output_path).parent
            images_dir = output_dir / "workflow_images"
            images_dir.mkdir(exist_ok=True)
            
            saved_count = 0
            
            # Save images from context
            for key, value in self.context.items():
                if isinstance(value, np.ndarray) and len(value.shape) >= 2:
                    # This is an image
                    image_filename = key if key.endswith(('.jpg', '.png', '.jpeg')) else f"{key}.jpg"
                    image_path = images_dir / image_filename
                    cv2.imwrite(str(image_path), value)
                    saved_count += 1
            
            if saved_count > 0:
                print(f"    ✓ Saved {saved_count} images to {images_dir}/")
            
        except Exception as e:
            print(f"    ⚠ Warning: Failed to save images separately: {e}")
