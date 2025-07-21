"""
Robot data structure for Duco robot arm monitoring.
This module contains the RobotData class that parses and stores robot state information.
"""

import struct
from dataclasses import dataclass
from typing import List


@dataclass
class RobotData:
    """
    Data structure to store robot monitoring information.
    Joint data arrays contain 7 floats corresponding to 7 robot joints.
    TCP data arrays contain 6 floats corresponding to X,Y,Z,Rx,Ry,Rz.
    """
    # Joint actual data
    jointActualPosition: List[float]    # bytes 0-27, unit: rad
    jointActualVelocity: List[float]    # bytes 28-55, unit: rad/s
    jointActualAccelera: List[float]    # bytes 56-83, unit: rad/s^2
    jointActualTorque: List[float]      # bytes 84-111, unit: Nm
    
    # Joint expected data
    jointExpectPosition: List[float]    # bytes 112-139, unit: rad
    jointExpectVelocity: List[float]    # bytes 140-167, unit: rad/s
    jointExpectAccelera: List[float]    # bytes 168-195, unit: rad/s^2
    jointExpectTorque: List[float]      # bytes 196-223, unit: Nm
    
    # Additional joint data
    jointActualTemperature: List[float] # bytes 224-251, unit: °C
    jointActualCurrent: List[float]     # bytes 252-279, unit: per thousand of rated current
    driverErrorID: List[int]            # bytes 280-307, uint
    driverState: List[int]              # bytes 308-335, uint
    
    # TCP actual data
    TCPActualPosition: List[float]      # bytes 368-391, X,Y,Z,Rx,Ry,Rz, unit: m,rad
    TCPActualVelocity: List[float]      # bytes 392-415, X,Y,Z,Rx,Ry,Rz, unit: m/s,rad/s
    TCPActualAccelera: List[float]      # bytes 416-439, X,Y,Z,Rx,Ry,Rz, unit: m/s^2,rad/s^2
    TCPActualTorque: List[float]        # bytes 440-463, X,Y,Z,Rx,Ry,Rz, unit: Nm
    
    # TCP expected data
    TCPExpectPosition: List[float]      # bytes 464-487, X,Y,Z,Rx,Ry,Rz, unit: m,rad
    TCPExpectVelocity: List[float]      # bytes 488-511, X,Y,Z,Rx,Ry,Rz, unit: m/s,rad/s
    TCPExpectAccelera: List[float]      # bytes 512-535, X,Y,Z,Rx,Ry,Rz, unit: m/s^2,rad/s^2
    TCPExpectTorque: List[float]        # bytes 536-559, X,Y,Z,Rx,Ry,Rz, unit: Nm
    
    # Base torque data
    baseActualTorque: List[float]       # bytes 560-583, X,Y,Z,Rx,Ry,Rz, unit: Nm
    baseExpectTorque: List[float]       # bytes 584-607, X,Y,Z,Rx,Ry,Rz, unit: Nm

    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotData':
        """
        Parse robot data from raw bytes.
        
        Args:
            data: Raw bytes received from robot (should be at least 608 bytes)
            
        Returns:
            RobotData instance with parsed values
        """
        if len(data) < 608:
            raise ValueError(f"Insufficient data length: {len(data)} bytes, need at least 608")
        
        # Parse joint data (28 bytes each = 7 floats * 4 bytes)
        jointActualPosition = list(struct.unpack("7f", data[0:28]))
        jointActualVelocity = list(struct.unpack("7f", data[28:56]))
        jointActualAccelera = list(struct.unpack("7f", data[56:84]))
        jointActualTorque = list(struct.unpack("7f", data[84:112]))
        jointExpectPosition = list(struct.unpack("7f", data[112:140]))
        jointExpectVelocity = list(struct.unpack("7f", data[140:168]))
        jointExpectAccelera = list(struct.unpack("7f", data[168:196]))
        jointExpectTorque = list(struct.unpack("7f", data[196:224]))
        
        # Parse additional joint data
        jointActualTemperature = list(struct.unpack("7f", data[224:252]))
        jointActualCurrent = list(struct.unpack("7f", data[252:280]))
        driverErrorID = list(struct.unpack("7I", data[280:308]))  # 7 unsigned ints
        driverState = list(struct.unpack("7I", data[308:336]))    # 7 unsigned ints
        
        # Skip reserved bytes 336-367 (32 bytes)
        
        # Parse TCP data (24 bytes each = 6 floats * 4 bytes)
        TCPActualPosition = list(struct.unpack("6f", data[368:392]))
        TCPActualVelocity = list(struct.unpack("6f", data[392:416]))
        TCPActualAccelera = list(struct.unpack("6f", data[416:440]))
        TCPActualTorque = list(struct.unpack("6f", data[440:464]))
        TCPExpectPosition = list(struct.unpack("6f", data[464:488]))
        TCPExpectVelocity = list(struct.unpack("6f", data[488:512]))
        TCPExpectAccelera = list(struct.unpack("6f", data[512:536]))
        TCPExpectTorque = list(struct.unpack("6f", data[536:560]))
        
        # Parse base torque data
        baseActualTorque = list(struct.unpack("6f", data[560:584]))
        baseExpectTorque = list(struct.unpack("6f", data[584:608]))
        
        return cls(
            jointActualPosition=jointActualPosition,
            jointActualVelocity=jointActualVelocity,
            jointActualAccelera=jointActualAccelera,
            jointActualTorque=jointActualTorque,
            jointExpectPosition=jointExpectPosition,
            jointExpectVelocity=jointExpectVelocity,
            jointExpectAccelera=jointExpectAccelera,
            jointExpectTorque=jointExpectTorque,
            jointActualTemperature=jointActualTemperature,
            jointActualCurrent=jointActualCurrent,
            driverErrorID=driverErrorID,
            driverState=driverState,
            TCPActualPosition=TCPActualPosition,
            TCPActualVelocity=TCPActualVelocity,
            TCPActualAccelera=TCPActualAccelera,
            TCPActualTorque=TCPActualTorque,
            TCPExpectPosition=TCPExpectPosition,
            TCPExpectVelocity=TCPExpectVelocity,
            TCPExpectAccelera=TCPExpectAccelera,
            TCPExpectTorque=TCPExpectTorque,
            baseActualTorque=baseActualTorque,
            baseExpectTorque=baseExpectTorque
        )

    def get_tcp_pose(self) -> List[float]:
        """
        Get TCP pose (x, y, z, rx, ry, rz) from TCP actual position.
        
        Returns:
            TCP actual position (X, Y, Z, Rx, Ry, Rz)
        """
        return self.TCPActualPosition
