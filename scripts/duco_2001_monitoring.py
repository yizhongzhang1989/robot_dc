#!/usr/bin/env python3  
"""  
This script connects via TCP to the robot at IP '192.168.1.10' on port 2001.  
The robot sends data at approximately 10Hz, with each data frame containing 1468 bytes.  

The data structure contains:
- bytes 0-27: jointActualPosition (7 floats, unit: rad)
- bytes 28-55: jointActualVelocity (7 floats, unit: rad/s)
- bytes 56-83: jointActualAccelera (7 floats, unit: rad/s^2)
- bytes 84-111: jointActualTorque (7 floats, unit: Nm)
- bytes 112-139: jointExpectPosition (7 floats, unit: rad)
- bytes 140-167: jointExpectVelocity (7 floats, unit: rad/s)
- bytes 168-195: jointExpectAccelera (7 floats, unit: rad/s^2)
- bytes 196-223: jointExpectTorque (7 floats, unit: Nm)
- bytes 224-251: jointActualTemperature (7 floats, unit: °C)
- bytes 252-279: jointActualCurrent (7 floats, unit: per thousand of rated current)
- bytes 280-307: driverErrorID (7 uint)
- bytes 308-335: driverState (7 uint)
- bytes 336-367: reserved
- bytes 368-391: TCPActualPosition (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m,rad)
- bytes 392-415: TCPActualVelocity (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m/s,rad/s)
- bytes 416-439: TCPActualAccelera (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m/s^2,rad/s^2)
- bytes 440-463: TCPActualTorque (6 floats, X,Y,Z,Rx,Ry,Rz, unit: Nm)
- bytes 464-487: TCPExpectPosition (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m,rad)
- bytes 488-511: TCPExpectVelocity (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m/s,rad/s)
- bytes 512-535: TCPExpectAccelera (6 floats, X,Y,Z,Rx,Ry,Rz, unit: m/s^2,rad/s^2)
- bytes 536-559: TCPExpectTorque (6 floats, X,Y,Z,Rx,Ry,Rz, unit: Nm)
- bytes 560-583: baseActualTorque (6 floats, X,Y,Z,Rx,Ry,Rz, unit: Nm)
- bytes 584-607: baseExpectTorque (6 floats, X,Y,Z,Rx,Ry,Rz, unit: Nm)

For each frame received, this script parses and displays comprehensive robot data
including joint and TCP actual/expected states, temperatures, currents, driver status,
and base torque information.
"""  
   
import socket  
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
        Updated to use actual TCP position data instead of joint position.
        
        Returns:
            TCP actual position (X, Y, Z, Rx, Ry, Rz)
        """
        return self.TCPActualPosition  
   
def main():  
    # HOST = '192.168.70.128'  # virtual machine IP
    HOST = '192.168.1.10'     # real robot
    PORT = 2001  
    FRAME_SIZE = 1468        # total size of each data frame in bytes  
    MIN_DATA_SIZE = 608      # minimum bytes needed for current data structure  
  
    # Create a TCP socket and connect to the robot.  
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
    try:  
        sock.connect((HOST, PORT))  
        print(f"Connected to {HOST}:{PORT}")  
    except Exception as e:  
        print(f"Failed to connect: {e}")  
        return  
  
    try:  
        while True:  
            # Read exactly FRAME_SIZE bytes from the socket.  
            data = b""  
            while len(data) < FRAME_SIZE:  
                packet = sock.recv(FRAME_SIZE - len(data))  
                if not packet:  
                    print("Connection closed by the sender.")  
                    return  
                data += packet  
  
            # Now process the frame if it has the expected size.  
            if len(data) == FRAME_SIZE:  
                # Parse the robot data using the structured format
                try:
                    robot_data = RobotData.from_bytes(data)
                    
                    # Print all robot data with formatted output
                    print("=" * 90)
                    print("ROBOT DATA:")
                    print("=" * 90)
                    
                    # Joint Actual Data
                    print("JOINT ACTUAL DATA:")
                    print(f"  Position (rad):     {' '.join(format(x, '.7g') for x in robot_data.jointActualPosition)}")
                    print(f"  Velocity (rad/s):   {' '.join(format(x, '.7g') for x in robot_data.jointActualVelocity)}")
                    print(f"  Acceleration (rad/s²): {' '.join(format(x, '.7g') for x in robot_data.jointActualAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.jointActualTorque)}")
                    print(f"  Temperature (°C):   {' '.join(format(x, '.7g') for x in robot_data.jointActualTemperature)}")
                    print(f"  Current (‰ rated):  {' '.join(format(x, '.7g') for x in robot_data.jointActualCurrent)}")
                    print(f"  Driver Error ID:    {' '.join(str(x) for x in robot_data.driverErrorID)}")
                    print(f"  Driver State:       {' '.join(str(x) for x in robot_data.driverState)}")
                    
                    print()
                    
                    # Joint Expected Data
                    print("JOINT EXPECTED DATA:")
                    print(f"  Position (rad):     {' '.join(format(x, '.7g') for x in robot_data.jointExpectPosition)}")
                    print(f"  Velocity (rad/s):   {' '.join(format(x, '.7g') for x in robot_data.jointExpectVelocity)}")
                    print(f"  Acceleration (rad/s²): {' '.join(format(x, '.7g') for x in robot_data.jointExpectAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.jointExpectTorque)}")
                    
                    print()
                    
                    # TCP Actual Data
                    print("TCP ACTUAL DATA:")
                    print(f"  Position (m,rad):   {' '.join(format(x, '.7g') for x in robot_data.TCPActualPosition)}")
                    print(f"  Velocity (m/s,rad/s): {' '.join(format(x, '.7g') for x in robot_data.TCPActualVelocity)}")
                    print(f"  Acceleration (m/s²,rad/s²): {' '.join(format(x, '.7g') for x in robot_data.TCPActualAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.TCPActualTorque)}")
                    
                    print()
                    
                    # TCP Expected Data
                    print("TCP EXPECTED DATA:")
                    print(f"  Position (m,rad):   {' '.join(format(x, '.7g') for x in robot_data.TCPExpectPosition)}")
                    print(f"  Velocity (m/s,rad/s): {' '.join(format(x, '.7g') for x in robot_data.TCPExpectVelocity)}")
                    print(f"  Acceleration (m/s²,rad/s²): {' '.join(format(x, '.7g') for x in robot_data.TCPExpectAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.TCPExpectTorque)}")
                    
                    print()
                    
                    # Base Torque Data
                    print("BASE TORQUE DATA:")
                    print(f"  Actual Torque (Nm): {' '.join(format(x, '.7g') for x in robot_data.baseActualTorque)}")
                    print(f"  Expected Torque (Nm): {' '.join(format(x, '.7g') for x in robot_data.baseExpectTorque)}")
                    
                    print()
                    
                    # TCP Pose (for reference - now using actual TCP data)
                    tcp_pose = robot_data.get_tcp_pose()
                    print(f"TCP POSE (X,Y,Z,Rx,Ry,Rz): {' '.join(format(x, '.7g') for x in tcp_pose)}")
                    print()
                    
                except ValueError as e:
                    print(f"Error parsing robot data: {e}")
            else:  
                print("Incomplete frame received. Skipping.")  
  
    except KeyboardInterrupt:  
        print("\nInterrupted by user. Exiting.")  
    except Exception as e:  
        print(f"An error occurred: {e}")  
    finally:  
        sock.close()  
        print("Socket closed.")  
   
if __name__ == "__main__":  
    main()  
