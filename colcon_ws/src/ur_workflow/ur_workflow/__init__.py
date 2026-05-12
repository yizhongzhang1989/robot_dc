"""
UR15 Workflow Package

A modular, workflow-driven architecture for robot operations.
Provides a flexible system where operations are defined in configuration files
and executed by specialized handlers.
"""

__version__ = "1.0.0"
__author__ = "Robot DC Team"

from .engine import WorkflowEngine
from .base import OperationHandler

__all__ = [
    'WorkflowEngine',
    'OperationHandler',
]
