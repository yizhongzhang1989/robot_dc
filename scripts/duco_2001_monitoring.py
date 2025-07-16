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

For each frame received, this script parses and displays all the robot data
including actual and expected joint positions, velocities, accelerations, and torques,
plus the TCP pose derived from the first 6 joint positions.
"""  
   
import socket  
import struct
from dataclasses import dataclass
from typing import List

@dataclass
class RobotData:
    """
    Data structure to store robot monitoring information.
    All joint data arrays contain 7 floats corresponding to 7 robot joints.
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
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotData':
        """
        Parse robot data from raw bytes.
        
        Args:
            data: Raw bytes received from robot (should be at least 224 bytes)
            
        Returns:
            RobotData instance with parsed values
        """
        if len(data) < 224:
            raise ValueError(f"Insufficient data length: {len(data)} bytes, need at least 224")
        
        # Parse each 28-byte section (7 floats * 4 bytes each)
        jointActualPosition = list(struct.unpack("7f", data[0:28]))
        jointActualVelocity = list(struct.unpack("7f", data[28:56]))
        jointActualAccelera = list(struct.unpack("7f", data[56:84]))
        jointActualTorque = list(struct.unpack("7f", data[84:112]))
        jointExpectPosition = list(struct.unpack("7f", data[112:140]))
        jointExpectVelocity = list(struct.unpack("7f", data[140:168]))
        jointExpectAccelera = list(struct.unpack("7f", data[168:196]))
        jointExpectTorque = list(struct.unpack("7f", data[196:224]))
        
        return cls(
            jointActualPosition=jointActualPosition,
            jointActualVelocity=jointActualVelocity,
            jointActualAccelera=jointActualAccelera,
            jointActualTorque=jointActualTorque,
            jointExpectPosition=jointExpectPosition,
            jointExpectVelocity=jointExpectVelocity,
            jointExpectAccelera=jointExpectAccelera,
            jointExpectTorque=jointExpectTorque
        )
    
    def get_tcp_pose(self) -> List[float]:
        """
        Get TCP pose (x, y, z, rx, ry, rz) from joint actual position.
        For backward compatibility with the original script.
        
        Returns:
            First 6 values from jointActualPosition
        """
        return self.jointActualPosition[:6]  
   
def main():  
    # HOST = '192.168.70.128'  # virtual machine IP
    HOST = '192.168.1.10'     # real robot
    PORT = 2001  
    FRAME_SIZE = 1468        # total size of each data frame in bytes  
    MIN_DATA_SIZE = 224      # minimum bytes needed for current data structure  
  
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
                    print("=" * 80)
                    print("ROBOT DATA:")
                    print("=" * 80)
                    
                    # Joint Actual Data
                    print("JOINT ACTUAL DATA:")
                    print(f"  Position (rad):     {' '.join(format(x, '.7g') for x in robot_data.jointActualPosition)}")
                    print(f"  Velocity (rad/s):   {' '.join(format(x, '.7g') for x in robot_data.jointActualVelocity)}")
                    print(f"  Acceleration (rad/s²): {' '.join(format(x, '.7g') for x in robot_data.jointActualAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.jointActualTorque)}")
                    
                    print()
                    
                    # Joint Expected Data
                    print("JOINT EXPECTED DATA:")
                    print(f"  Position (rad):     {' '.join(format(x, '.7g') for x in robot_data.jointExpectPosition)}")
                    print(f"  Velocity (rad/s):   {' '.join(format(x, '.7g') for x in robot_data.jointExpectVelocity)}")
                    print(f"  Acceleration (rad/s²): {' '.join(format(x, '.7g') for x in robot_data.jointExpectAccelera)}")
                    print(f"  Torque (Nm):        {' '.join(format(x, '.7g') for x in robot_data.jointExpectTorque)}")
                    
                    print()
                    
                    # TCP Pose (for reference)
                    tcp_pose = robot_data.get_tcp_pose()
                    print(f"TCP POSE (x,y,z,rx,ry,rz): {' '.join(format(x, '.7g') for x in tcp_pose)}")
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
