#!/usr/bin/env python3
"""
Base classes for workflow operations
"""

import json
from typing import Dict, Any


class OperationHandler:
    """
    Base class for operation handlers
    All handlers must implement the execute method
    """
    
    def execute(self, operation: Dict[str, Any], context: Dict[str, Any], 
                previous_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute the operation
        
        Args:
            operation: Operation definition from workflow
            context: Shared context (can be read and written)
            previous_results: Results from previous operations
            
        Returns:
            Result dictionary with at least 'status' key
        """
        raise NotImplementedError("Subclasses must implement execute()")
    
    def _resolve_parameter(self, param: Any, context: Dict[str, Any]) -> Any:
        """
        Resolve parameter value from context if needed
        
        Args:
            param: Parameter value (can be literal or reference)
            context: Context dictionary
            
        Returns:
            Resolved parameter value
        """
        if isinstance(param, str):
            # Check for context reference: ${context.key}
            if param.startswith('${') and param.endswith('}'):
                key = param[2:-1].replace('context.', '')
                return context.get(key)
            
            # Check for file reference: load_from_json:path:key
            if param.startswith('load_from_json:'):
                parts = param.split(':', 2)
                if len(parts) == 3:
                    _, file_path, key = parts
                    return self._load_from_json(file_path, key)
        
        return param
    
    def _load_from_json(self, file_path: str, key: str) -> Any:
        """
        Load value from JSON file
        
        Args:
            file_path: Path to JSON file
            key: Key to extract
            
        Returns:
            Value from JSON file
        """
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            return data.get(key)
        except Exception as e:
            print(f"✗ Failed to load from JSON: {e}")
            return None
