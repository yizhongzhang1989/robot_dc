#!/usr/bin/env python3  
"""  
This script connects via TCP to the robot at IP '192.168.1.10' on port 2001.  
The robot sends data at approximately 10Hz, with each data frame containing 1468 bytes.  

The complete data structure contains:
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
- bytes 608-631: activeToolCoordSystem (6 floats)
- bytes 632-655: activeWorkpieceCoordSystem (6 floats)
- bytes 656-659: blendedSpeed (float)
- byte 660: globalSpeed (byte, percentage)
- byte 661: jogSpeed (byte, percentage)
- bytes 662-719: reserved
- bytes 720-727: functionalDigitalIOInput (8 bytes, FDI1-FDI8)
- bytes 728-735: functionalDigitalIOOutput (8 bytes, FDI1-FDI8)
- bytes 736-751: digitalIOInput (16 bytes, DI1-DI16)
- bytes 752-767: digitalIOOutput (16 bytes, DI1-DI16)
- bytes 768-799: analogInput (8 floats)
- bytes 800-831: analogOutput (8 floats)
- bytes 832-959: floatRegisterInput (32 floats)
- bytes 960-1087: floatRegisterOutput (32 floats)
- bytes 1088-1103: functionalBoolRegisterInput (16 bytes)
- bytes 1104-1119: functionalBoolRegisterOutput (16 bytes)
- bytes 1120-1183: boolRegisterInput (64 bytes)
- bytes 1184-1247: boolRegisterOutput (64 bytes)
- bytes 1248-1311: wordRegisterInput (64 chars)
- bytes 1312-1375: wordRegisterOutput (64 chars)
- bytes 1376-1406: reserved
- byte 1407: simulationMode (0=real, 1=simulation)
- bytes 1408-1415: toolIOInput (8 bytes)
- bytes 1416-1423: toolIOOutput (2 floats)
- bytes 1424-1431: toolAnalogInput (2 floats)
- bytes 1432-1439: toolAnalogOutput (2 floats)
- bytes 1440-1441: toolButtonStatus (2 bytes,True=1,false=0. Button S and Button T)
- bytes 1442-1447: reserved
- byte 1448: robotOperationMode (0=Manual, 1=Auto, 2=Remote)
- byte 1449: robotStatus (0=SR_Start,1=SR_Initialize,2=SR_Logout,3=SR_Login,4=SR_PowerOff, 5=SR_Disable/PowerOn,6=SR_Enable)
- byte 1450: robotProgramRunStatus (0-5 various states)
- byte 1451: safetyMonitorStatus (0-15 various states)
- byte 1452: collisionDetectionTrigger (1 means collision detected)
- byte 1453: collisionAxis  (1-6 means collision on joint 1-6)
- bytes 1454-1455: reserved
- bytes 1456-1459: robotErrorCode (uint)
- bytes 1460-1467: reserved

For each frame received, this script parses and displays comprehensive robot data
including joint and TCP actual/expected states, temperatures, currents, driver status,
IO states, registers, tool data, and robot status information.
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
    
    # Tool and workpiece coordinate systems
    activeToolCoordSystem: List[float]  # bytes 608-631, 6 floats
    activeWorkpieceCoordSystem: List[float]  # bytes 632-655, 6 floats
    
    # Speed settings
    blendedSpeed: float                 # bytes 656-659, unit: m/s or rad/s
    globalSpeed: int                    # byte 660, percentage
    jogSpeed: int                       # byte 661, percentage
    
    # Digital IO
    functionalDigitalIOInput: List[int] # bytes 720-727, FDI1-FDI8
    functionalDigitalIOOutput: List[int] # bytes 728-735
    digitalIOInput: List[int]           # bytes 736-751, DI1-DI16
    digitalIOOutput: List[int]          # bytes 752-767
    
    # Analog IO
    analogInput: List[float]            # bytes 768-799, 8 floats
    analogOutput: List[float]           # bytes 800-831, 8 floats
    
    # Float registers
    floatRegisterInput: List[float]     # bytes 832-959, 32 floats
    floatRegisterOutput: List[float]    # bytes 960-1087, 32 floats
    
    # Bool registers
    functionalBoolRegisterInput: List[int]  # bytes 1088-1103, 16 bytes
    functionalBoolRegisterOutput: List[int] # bytes 1104-1119, 16 bytes
    boolRegisterInput: List[int]        # bytes 1120-1183, 64 bytes
    boolRegisterOutput: List[int]       # bytes 1184-1247, 64 bytes
    
    # Word registers
    wordRegisterInput: List[int]        # bytes 1248-1311, 64 chars
    wordRegisterOutput: List[int]       # bytes 1312-1375, 64 chars
    
    # Robot status
    simulationMode: int                 # byte 1407, 0=real, 1=simulation
    
    # Tool IO
    toolIOInput: List[int]              # bytes 1408-1415, 8 bytes
    toolIOOutput: List[float]           # bytes 1416-1423, 2 floats
    toolAnalogInput: List[float]        # bytes 1424-1431, 2 floats
    toolAnalogOutput: List[float]       # bytes 1432-1439, 2 floats
    toolButtonStatus: List[int]         # bytes 1440-1441, 2 bytes
    
    # Robot operation mode and status
    robotOperationMode: int             # byte 1448, 0=Manual, 1=Auto, 2=Remote
    robotStatus: int                    # byte 1449, robot state
    robotProgramRunStatus: int          # byte 1450, program run status
    safetyMonitorStatus: int            # byte 1451, safety monitor status
    collisionDetectionTrigger: int      # byte 1452, collision detection signal
    collisionAxis: int                  # byte 1453, collision axis
    
    # Error code
    robotErrorCode: int                 # bytes 1456-1459, uint error code
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotData':
        """
        Parse robot data from raw bytes.
        
        Args:
            data: Raw bytes received from robot (should be 1468 bytes)
            
        Returns:
            RobotData instance with parsed values
        """
        if len(data) < 1468:
            raise ValueError(f"Insufficient data length: {len(data)} bytes, need 1468")
        
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
        
        # Parse tool and workpiece coordinate systems
        activeToolCoordSystem = list(struct.unpack("6f", data[608:632]))
        activeWorkpieceCoordSystem = list(struct.unpack("6f", data[632:656]))
        
        # Parse speed settings
        blendedSpeed = struct.unpack("f", data[656:660])[0]
        globalSpeed = struct.unpack("B", data[660:661])[0]
        jogSpeed = struct.unpack("B", data[661:662])[0]
        
        # Skip reserved bytes 662-719 (58 bytes)
        
        # Parse digital IO
        functionalDigitalIOInput = list(struct.unpack("8B", data[720:728]))
        functionalDigitalIOOutput = list(struct.unpack("8B", data[728:736]))
        digitalIOInput = list(struct.unpack("16B", data[736:752]))
        digitalIOOutput = list(struct.unpack("16B", data[752:768]))
        
        # Parse analog IO
        analogInput = list(struct.unpack("8f", data[768:800]))
        analogOutput = list(struct.unpack("8f", data[800:832]))
        
        # Parse float registers
        floatRegisterInput = list(struct.unpack("32f", data[832:960]))
        floatRegisterOutput = list(struct.unpack("32f", data[960:1088]))
        
        # Parse bool registers
        functionalBoolRegisterInput = list(struct.unpack("16B", data[1088:1104]))
        functionalBoolRegisterOutput = list(struct.unpack("16B", data[1104:1120]))
        boolRegisterInput = list(struct.unpack("64B", data[1120:1184]))
        boolRegisterOutput = list(struct.unpack("64B", data[1184:1248]))
        
        # Parse word registers
        wordRegisterInput = list(struct.unpack("64B", data[1248:1312]))
        wordRegisterOutput = list(struct.unpack("64B", data[1312:1376]))
        
        # Skip reserved bytes 1376-1406 (31 bytes)
        
        # Parse robot status
        simulationMode = struct.unpack("B", data[1407:1408])[0]
        
        # Parse tool IO
        toolIOInput = list(struct.unpack("8B", data[1408:1416]))
        toolIOOutput = list(struct.unpack("2f", data[1416:1424]))
        toolAnalogInput = list(struct.unpack("2f", data[1424:1432]))
        toolAnalogOutput = list(struct.unpack("2f", data[1432:1440]))
        toolButtonStatus = list(struct.unpack("2B", data[1440:1442]))
        
        # Skip reserved bytes 1442-1447 (6 bytes)
        
        # Parse robot operation mode and status
        robotOperationMode = struct.unpack("B", data[1448:1449])[0]
        robotStatus = struct.unpack("B", data[1449:1450])[0]
        robotProgramRunStatus = struct.unpack("B", data[1450:1451])[0]
        safetyMonitorStatus = struct.unpack("B", data[1451:1452])[0]
        collisionDetectionTrigger = struct.unpack("B", data[1452:1453])[0]
        collisionAxis = struct.unpack("B", data[1453:1454])[0]
        
        # Skip reserved bytes 1454-1455 (2 bytes)
        
        # Parse error code
        robotErrorCode = struct.unpack("I", data[1456:1460])[0]
        
        # Skip final reserved bytes 1460-1467 (8 bytes)
        
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
            baseExpectTorque=baseExpectTorque,
            activeToolCoordSystem=activeToolCoordSystem,
            activeWorkpieceCoordSystem=activeWorkpieceCoordSystem,
            blendedSpeed=blendedSpeed,
            globalSpeed=globalSpeed,
            jogSpeed=jogSpeed,
            functionalDigitalIOInput=functionalDigitalIOInput,
            functionalDigitalIOOutput=functionalDigitalIOOutput,
            digitalIOInput=digitalIOInput,
            digitalIOOutput=digitalIOOutput,
            analogInput=analogInput,
            analogOutput=analogOutput,
            floatRegisterInput=floatRegisterInput,
            floatRegisterOutput=floatRegisterOutput,
            functionalBoolRegisterInput=functionalBoolRegisterInput,
            functionalBoolRegisterOutput=functionalBoolRegisterOutput,
            boolRegisterInput=boolRegisterInput,
            boolRegisterOutput=boolRegisterOutput,
            wordRegisterInput=wordRegisterInput,
            wordRegisterOutput=wordRegisterOutput,
            simulationMode=simulationMode,
            toolIOInput=toolIOInput,
            toolIOOutput=toolIOOutput,
            toolAnalogInput=toolAnalogInput,
            toolAnalogOutput=toolAnalogOutput,
            toolButtonStatus=toolButtonStatus,
            robotOperationMode=robotOperationMode,
            robotStatus=robotStatus,
            robotProgramRunStatus=robotProgramRunStatus,
            safetyMonitorStatus=safetyMonitorStatus,
            collisionDetectionTrigger=collisionDetectionTrigger,
            collisionAxis=collisionAxis,
            robotErrorCode=robotErrorCode
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
                    
                    # Coordinate Systems
                    print("COORDINATE SYSTEMS:")
                    print(f"  Active Tool Coord:  {' '.join(format(x, '.7g') for x in robot_data.activeToolCoordSystem)}")
                    print(f"  Active Workpiece Coord: {' '.join(format(x, '.7g') for x in robot_data.activeWorkpieceCoordSystem)}")
                    
                    print()
                    
                    # Speed Settings
                    print("SPEED SETTINGS:")
                    print(f"  Blended Speed:      {robot_data.blendedSpeed:.7g}")
                    print(f"  Global Speed:       {robot_data.globalSpeed}%")
                    print(f"  Jog Speed:          {robot_data.jogSpeed}%")
                    
                    print()
                    
                    # Digital IO
                    print("DIGITAL IO:")
                    print(f"  Functional DI Input: {' '.join(str(x) for x in robot_data.functionalDigitalIOInput)}")
                    print(f"  Functional DI Output: {' '.join(str(x) for x in robot_data.functionalDigitalIOOutput)}")
                    print(f"  Digital Input:      {' '.join(str(x) for x in robot_data.digitalIOInput)}")
                    print(f"  Digital Output:     {' '.join(str(x) for x in robot_data.digitalIOOutput)}")
                    
                    print()
                    
                    # Analog IO
                    print("ANALOG IO:")
                    print(f"  Analog Input:       {' '.join(format(x, '.7g') for x in robot_data.analogInput)}")
                    print(f"  Analog Output:      {' '.join(format(x, '.7g') for x in robot_data.analogOutput)}")
                    
                    print()
                    
                    # Registers (only show first few to avoid too much output)
                    print("REGISTERS (first 8 values shown):")
                    print(f"  Float Reg Input:    {' '.join(format(x, '.7g') for x in robot_data.floatRegisterInput[:8])}")
                    print(f"  Float Reg Output:   {' '.join(format(x, '.7g') for x in robot_data.floatRegisterOutput[:8])}")
                    print(f"  Func Bool Reg In:   {' '.join(str(x) for x in robot_data.functionalBoolRegisterInput[:8])}")
                    print(f"  Func Bool Reg Out:  {' '.join(str(x) for x in robot_data.functionalBoolRegisterOutput[:8])}")
                    print(f"  Bool Reg Input:     {' '.join(str(x) for x in robot_data.boolRegisterInput[:8])}")
                    print(f"  Bool Reg Output:    {' '.join(str(x) for x in robot_data.boolRegisterOutput[:8])}")
                    print(f"  Word Reg Input:     {' '.join(str(x) for x in robot_data.wordRegisterInput[:8])}")
                    print(f"  Word Reg Output:    {' '.join(str(x) for x in robot_data.wordRegisterOutput[:8])}")
                    
                    print()
                    
                    # Tool IO
                    print("TOOL IO:")
                    print(f"  Tool IO Input:      {' '.join(str(x) for x in robot_data.toolIOInput)}")
                    print(f"  Tool IO Output:     {' '.join(format(x, '.7g') for x in robot_data.toolIOOutput)}")
                    print(f"  Tool Analog Input:  {' '.join(format(x, '.7g') for x in robot_data.toolAnalogInput)}")
                    print(f"  Tool Analog Output: {' '.join(format(x, '.7g') for x in robot_data.toolAnalogOutput)}")
                    print(f"  Tool Button Status: {' '.join(str(x) for x in robot_data.toolButtonStatus)}")
                    
                    print()
                    
                    # Robot Status
                    print("ROBOT STATUS:")
                    operation_modes = {0: "Manual", 1: "Auto", 2: "Remote"}
                    robot_states = {0: "Start", 1: "Initialize", 2: "Logout", 3: "Login", 
                                   4: "PowerOff", 5: "Disable/PowerOn", 6: "Enable"}
                    program_states = {0: "Stopped", 1: "Stopping", 2: "Running", 
                                     3: "Paused", 4: "Pausing", 5: "TaskRunning"}
                    safety_states = {0: "INIT", 2: "WAIT", 3: "CONFIG", 4: "POWER_OFF", 
                                   5: "RUN", 6: "RECOVERY", 7: "STOP2", 8: "STOP1", 
                                   9: "STOP0", 10: "MODEL", 12: "REDUCE", 13: "BOOT", 
                                   14: "FAIL", 15: "UPDATE"}
                    
                    print(f"  Simulation Mode:    {'Simulation' if robot_data.simulationMode else 'Real Robot'}")
                    print(f"  Operation Mode:     {operation_modes.get(robot_data.robotOperationMode, 'Unknown')} ({robot_data.robotOperationMode})")
                    print(f"  Robot Status:       {robot_states.get(robot_data.robotStatus, 'Unknown')} ({robot_data.robotStatus})")
                    print(f"  Program Run Status: {program_states.get(robot_data.robotProgramRunStatus, 'Unknown')} ({robot_data.robotProgramRunStatus})")
                    print(f"  Safety Monitor:     {safety_states.get(robot_data.safetyMonitorStatus, 'Unknown')} ({robot_data.safetyMonitorStatus})")
                    print(f"  Collision Detection: {'Triggered' if robot_data.collisionDetectionTrigger else 'Normal'}")
                    print(f"  Collision Axis:     {robot_data.collisionAxis}")
                    print(f"  Error Code:         {robot_data.robotErrorCode}")
                    
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
