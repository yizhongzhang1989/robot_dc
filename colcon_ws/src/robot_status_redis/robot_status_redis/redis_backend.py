#!/usr/bin/env python3
"""
Redis Backend for Robot Status

Internal interface for Redis-based status storage.
NOT intended for direct user import - use RobotStatusClient instead.

This module provides thread-safe, low-latency status storage using Redis.
"""

import redis
import json
import pickle
import base64
import time
from typing import Any, Dict, List, Optional, Tuple
from pathlib import Path


class RedisBackend:
    """Redis backend for robot status storage.
    
    This is an internal interface. Users should use RobotStatusClient instead.
    
    Data Persistence:
        - Redis data persists after process termination
        - Data remains until Redis server restart (unless RDB/AOF persistence configured)
        - Keys can be explicitly deleted or set to expire
    
    Namespace Isolation:
        - Use 'key_prefix' to isolate data between applications/processes
        - Default: 'robot_status' (shared by all robot_status_redis users)
        - Example: key_prefix='my_app' creates keys like 'my_app:robot1:pose'
        - Different processes can use different key_prefix values for isolation
    """
    
    def __init__(
        self,
        host: str = 'localhost',
        port: int = 6379,
        db: int = 0,
        password: Optional[str] = None,
        key_prefix: str = 'robot_status',
        max_retries: int = 3,
        retry_delay: float = 0.1
    ):
        """Initialize Redis backend.
        
        Args:
            host: Redis server host
            port: Redis server port
            db: Redis database number (0-15, use different db for complete isolation)
            password: Redis password (if required)
            key_prefix: Prefix for all Redis keys (default: 'robot_status')
                       Use different prefixes to isolate data between applications
            max_retries: Maximum connection retry attempts
            retry_delay: Delay between retries in seconds
        """
        self.host = host
        self.port = port
        self.db = db
        self.password = password
        self.key_prefix = key_prefix
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        
        self._client: Optional[redis.Redis] = None
        self._connect()
    
    def _connect(self):
        """Establish connection to Redis server with retry logic."""
        for attempt in range(self.max_retries):
            try:
                self._client = redis.Redis(
                    host=self.host,
                    port=self.port,
                    db=self.db,
                    password=self.password,
                    decode_responses=True,  # Decode bytes to strings
                    socket_connect_timeout=1,
                    socket_timeout=1
                )
                # Test connection
                self._client.ping()
                return
            except (redis.ConnectionError, redis.TimeoutError) as e:
                if attempt < self.max_retries - 1:
                    time.sleep(self.retry_delay)
                else:
                    raise ConnectionError(
                        f"Failed to connect to Redis at {self.host}:{self.port} "
                        f"after {self.max_retries} attempts: {e}"
                    )
    
    def is_connected(self) -> bool:
        """Check if connected to Redis.
        
        Returns:
            True if connected, False otherwise
        """
        try:
            if self._client is None:
                return False
            self._client.ping()
            return True
        except (redis.ConnectionError, redis.TimeoutError):
            return False
    
    def _make_key(self, namespace: str, key: str) -> str:
        """Create Redis key from namespace and key.
        
        Args:
            namespace: Namespace (e.g., 'ur15', 'duco')
            key: Status key (e.g., 'tcp_pos')
        
        Returns:
            Redis key string (e.g., 'robot_status:ur15:tcp_pos')
        """
        return f"{self.key_prefix}:{namespace}:{key}"
    
    def _parse_key(self, redis_key: str) -> Optional[Tuple[str, str]]:
        """Parse Redis key into namespace and key.
        
        Args:
            redis_key: Redis key string
        
        Returns:
            Tuple of (namespace, key) or None if invalid format
        """
        prefix = f"{self.key_prefix}:"
        if not redis_key.startswith(prefix):
            return None
        
        parts = redis_key.split(':', 2)
        if len(parts) != 3:
            return None
        
        return parts[1], parts[2]
    
    def _serialize_value(self, value: Any) -> str:
        """Serialize value to pickle base64 string (same format as ROS2 service).
        
        Args:
            value: Python object to serialize
        
        Returns:
            Base64-encoded pickle string
        """
        pickled = pickle.dumps(value)
        return base64.b64encode(pickled).decode('ascii')
    
    def _deserialize_value(self, value_str: str) -> Any:
        """Deserialize pickle base64 string to Python object.
        
        Args:
            value_str: Base64-encoded pickle string
        
        Returns:
            Deserialized Python object
        """
        pickled = base64.b64decode(value_str.encode('ascii'))
        return pickle.loads(pickled)
    
    def _create_storage_dict(self, value: Any) -> Dict[str, Any]:
        """Create storage dictionary with pickle and optional JSON.
        
        Args:
            value: Python object to store
        
        Returns:
            Dict with {"pickle": "...", "json": {...}} or just {"pickle": "..."}
        """
        result = {"pickle": self._serialize_value(value)}
        
        # Try to add JSON representation
        try:
            # Try direct JSON serialization first
            json_str = json.dumps(value)
            result["json"] = json.loads(json_str)
        except (TypeError, ValueError):
            # Direct serialization failed, try tolist() method
            if hasattr(value, 'tolist'):
                try:
                    list_obj = value.tolist()
                    json_str = json.dumps(list_obj)
                    result["json"] = json.loads(json_str)
                except (TypeError, ValueError, AttributeError):
                    # tolist() also failed, no JSON representation
                    pass
        
        return result
    
    def set_status(self, namespace: str, key: str, value: Any) -> bool:
        """Set status value with timestamp.
        
        Args:
            namespace: Namespace (e.g., 'ur15', 'duco')
            key: Status key (e.g., 'tcp_pos')
            value: Value to store (any Python object)
        
        Returns:
            True if successful, False otherwise
        """
        try:
            redis_key = self._make_key(namespace, key)
            pickle_str = self._serialize_value(value)
            
            # Store dict with pickle_str and timestamp as JSON
            data = {
                'pickle': pickle_str,
                'timestamp': time.time()
            }
            self._client.set(redis_key, json.dumps(data))
            return True
        except Exception:
            return False
    
    def get_status(self, namespace: str, key: str) -> Tuple[bool, Any]:
        """Get status value.
        
        Args:
            namespace: Namespace (e.g., 'ur15', 'duco')
            key: Status key (e.g., 'tcp_pos')
        
        Returns:
            Tuple of (success, value). If not found, returns (False, None)
        """
        try:
            redis_key = self._make_key(namespace, key)
            data_str = self._client.get(redis_key)
            
            if data_str is None:
                return False, None
            
            # Parse JSON to get dict with pickle and timestamp
            data = json.loads(data_str)
            pickle_str = data['pickle']
            
            # Deserialize pickle string
            value = self._deserialize_value(pickle_str)
            return True, value
        except Exception:
            return False, None
    
    def delete_status(self, namespace: str, key: str) -> bool:
        """Delete status value.
        
        Args:
            namespace: Namespace (e.g., 'ur15', 'duco')
            key: Status key (e.g., 'tcp_pos')
        
        Returns:
            True if deleted, False if not found or error
        """
        try:
            redis_key = self._make_key(namespace, key)
            result = self._client.delete(redis_key)
            return result > 0
        except Exception:
            return False
    
    def list_status(self, namespace: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
        """List all status values, optionally filtered by namespace.
        
        Args:
            namespace: Optional namespace filter
        
        Returns:
            Dict of {namespace: {key: storage_dict}}
            where storage_dict = {"pickle": "...", "json": {...}} or {"pickle": "..."}
        """
        try:
            # Determine search pattern
            if namespace:
                pattern = f"{self.key_prefix}:{namespace}:*"
            else:
                pattern = f"{self.key_prefix}:*"
            
            # Get all matching keys
            keys = self._client.keys(pattern)
            
            # Build result tree
            result = {}
            for redis_key in keys:
                parsed = self._parse_key(redis_key)
                if parsed is None:
                    continue
                
                ns, key = parsed
                
                # Filter by namespace if specified (already handled in pattern, but double-check)
                if namespace and ns != namespace:
                    continue
                
                # Initialize namespace dict
                if ns not in result:
                    result[ns] = {}
                
                # Get data (JSON string containing dict)
                data_str = self._client.get(redis_key)
                if data_str:
                    try:
                        # Parse JSON to get dict with pickle and timestamp
                        data = json.loads(data_str)
                        result[ns][key] = data  # Returns {'pickle': ..., 'timestamp': ...}
                    except json.JSONDecodeError:
                        # Skip invalid data
                        pass
            
            return result
        except Exception:
            return {}
    
    def close(self):
        """Close Redis connection."""
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None


# Singleton instance for shared access
_redis_backend: Optional[RedisBackend] = None


def get_redis_backend(
    host: str = 'localhost',
    port: int = 6379,
    db: int = 0,
    password: Optional[str] = None,
    key_prefix: str = 'robot_status',
    reconnect: bool = False
) -> Optional[RedisBackend]:
    """Get or create Redis backend singleton.
    
    NOTE: Singleton is shared - first call sets key_prefix for all subsequent uses.
    For different prefixes, create separate RedisBackend instances directly.
    
    Args:
        host: Redis server host
        port: Redis server port
        db: Redis database number
        password: Redis password (if required)
        key_prefix: Prefix for Redis keys (default: 'robot_status')
        reconnect: Force reconnection even if instance exists
    
    Returns:
        RedisBackend instance if successful, None if connection failed
    """
    global _redis_backend
    
    if reconnect and _redis_backend is not None:
        _redis_backend.close()
        _redis_backend = None
    
    if _redis_backend is None:
        try:
            _redis_backend = RedisBackend(
                host=host,
                port=port,
                db=db,
                password=password,
                key_prefix=key_prefix
            )
        except ConnectionError:
            return None
    
    return _redis_backend


def is_redis_available(
    host: str = 'localhost',
    port: int = 6379,
    db: int = 0,
    password: Optional[str] = None
) -> bool:
    """Check if Redis is available.
    
    Args:
        host: Redis server host
        port: Redis server port
        db: Redis database number
        password: Redis password (if required)
    
    Returns:
        True if Redis is available, False otherwise
    """
    try:
        backend = get_redis_backend(host, port, db, password)
        return backend is not None and backend.is_connected()
    except Exception:
        return False
