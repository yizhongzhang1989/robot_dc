#!/usr/bin/env python3
"""
High-level control interface for Robotiq 2F-140 Gripper.
"""

import time
from typing import Optional
from .rs485_client import RS485Client


class Robotiq2f140Gripper:
    """
    High-level control interface for Robotiq 2F-140 Gripper.
    
    Modbus Register Map:
    - 0x03E8 (1000): Action Request Register
    - 0x07D0 (2000): Gripper Status Register
    """
    
    ACTION_REQUEST_REG = 0x03E8
    GRIPPER_STATUS_REG = 0x07D0
    
    def __init__(self, device_id: int = 9, 
                 rs485_client: Optional[RS485Client] = None,
                 host: str = "192.168.1.15", 
                 port: int = 54321):
        """
        Initialize Robotiq 2F-140 Gripper controller.
        
        Args:
            device_id: Modbus device ID (default: 9)
            rs485_client: Existing RS485Client instance, or None to create new one
            host: RS485 gateway IP address
            port: RS485 gateway port
        """
        self.device_id = device_id
        self._owns_client = rs485_client is None
        
        if rs485_client is None:
            self.rs485 = RS485Client(host, port)
            self.rs485.connect()
        else:
            self.rs485 = rs485_client
        
        self._is_activated = False
    
    def activate(self, timeout: float = 3.0) -> bool:
        """
        Activate the gripper. Must be called before any motion commands.
        
        Returns:
            True if activation successful
        """
        # Reset gripper
        response = self.rs485.send_modbus_request(
            device_id=self.device_id,
            function_code=16,
            address=self.ACTION_REQUEST_REG,
            values=[0x0000, 0x0000, 0x0000]
        )
        
        if not response:
            return False
        
        time.sleep(0.5)
        
        # Activate gripper
        response = self.rs485.send_modbus_request(
            device_id=self.device_id,
            function_code=16,
            address=self.ACTION_REQUEST_REG,
            values=[0x0100, 0x0000, 0x0000]
        )
        
        if not response:
            return False
        
        # Wait for activation
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and status.get('activated', False):
                self._is_activated = True
                return True
            time.sleep(0.1)
        
        return False
    
    def get_status(self) -> Optional[dict]:
        """
        Read gripper status.
        
        Returns:
            Dictionary containing status information
        """
        response = self.rs485.send_modbus_request(
            device_id=self.device_id,
            function_code=4,
            address=self.GRIPPER_STATUS_REG,
            values=2
        )
        
        if not response:
            return None
        
        parsed = RS485Client.parse_modbus_response(response, function_code=4)
        
        if parsed.get('error') or 'registers' not in parsed:
            return None
        
        registers = parsed['registers']
        if len(registers) < 2:
            return None
        
        status_byte = registers[0] >> 8
        position_byte = registers[0] & 0xFF
        
        status = {
            'activated': bool(status_byte & 0x01),
            'moving': bool(status_byte & 0x08),
            'object_detected': bool((status_byte >> 6) & 0x03),
            'fault': bool(status_byte & 0x0F),
            'position': position_byte,
            'force': 0,  # Not directly available in status
            'raw_registers': registers
        }
        
        return status
    
    def move_to_position(self, position: int, speed: int = 255, 
                        force: int = 255, wait: bool = True) -> bool:
        """
        Move gripper to specified position.
        
        Args:
            position: Target position (0=open, 255=closed)
            speed: Closing speed (0-255)
            force: Gripping force (0-255)
            wait: Wait for motion to complete
        
        Returns:
            True if command sent successfully
        """
        position = max(0, min(255, position))
        speed = max(0, min(255, speed))
        force = max(0, min(255, force))
        
        action_byte = 0x0900  # rACT=1, rGTO=1
        position_byte = position
        speed_force_byte = (speed << 8) | force
        
        response = self.rs485.send_modbus_request(
            device_id=self.device_id,
            function_code=16,
            address=self.ACTION_REQUEST_REG,
            values=[action_byte, position_byte, speed_force_byte]
        )
        
        if not response:
            return False
        
        if wait:
            return self.wait_for_motion_complete()
        
        return True
    
    def open(self, speed: int = 255, wait: bool = True) -> bool:
        """Fully open the gripper."""
        return self.move_to_position(0, speed=speed, force=0, wait=wait)
    
    def close(self, speed: int = 255, force: int = 255, wait: bool = True) -> bool:
        """Fully close the gripper."""
        return self.move_to_position(255, speed=speed, force=force, wait=wait)
    
    def wait_for_motion_complete(self, timeout: float = 5.0) -> bool:
        """Wait for gripper motion to complete."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if status and not status['moving']:
                return True
            time.sleep(0.05)
        return False
    
    def stop(self) -> bool:
        """Stop gripper motion immediately."""
        response = self.rs485.send_modbus_request(
            device_id=self.device_id,
            function_code=16,
            address=self.ACTION_REQUEST_REG,
            values=[0x0100, 0x0000, 0x0000]
        )
        return response is not None
    
    def disconnect(self):
        """Disconnect if we own the client."""
        if self._owns_client:
            self.rs485.disconnect()
