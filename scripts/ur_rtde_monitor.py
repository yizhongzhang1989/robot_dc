#!/usr/bin/env python3
"""
RTDE Monitor Script
Read robot real-time data using Universal Robots RTDE Python Client Library

Reference: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html
"""

import sys
import time
import argparse
from rtde import RTDE
from rtde.rtde_config import ConfigFile


# Robot default IP and port
DEFAULT_ROBOT_IP = "192.168.1.15"
RTDE_PORT = 30004


def connect_rtde(host, port=RTDE_PORT):
    """
    Connect to RTDE interface
    
    Args:
        host: Robot IP address
        port: RTDE port (default 30004)
    
    Returns:
        RTDE connection object
    """
    print(f"Connecting to robot {host}:{port}...")
    con = RTDE(host, port)
    
    # Connect to RTDE interface (also negotiates protocol version)
    con.connect()
    
    # Get controller version
    version = con.get_controller_version()
    print(f"Connected successfully!")
    print(f"Controller version: {version[0]}.{version[1]}.{version[2]}.{version[3]}")
    
    return con


def setup_output_recipe(con, variables, frequency=125):
    """
    Setup output data recipe
    
    Args:
        con: RTDE connection object
        variables: List of variables to read
        frequency: Output frequency (Hz), default 125
    
    Returns:
        Configuration result
    """
    print(f"\nSetting up output data (frequency: {frequency} Hz)...")
    print(f"Reading variables: {', '.join(variables)}")
    
    # Setup output recipe
    if not con.send_output_setup(variables, frequency=frequency):
        print("Error: Unable to configure output data")
        sys.exit(1)
    
    print("Output recipe configured successfully!")
    return True


def monitor_robot_data(con, print_interval=0.5):
    """
    Monitor robot data
    
    Args:
        con: RTDE connection object
        print_interval: Print interval (seconds)
    """
    print(f"\nStarting robot data monitoring (Press Ctrl+C to stop)...")
    print("=" * 80)
    
    # Start data synchronization
    if not con.send_start():
        print("Error: Unable to start data synchronization")
        sys.exit(1)
    
    start_time = time.time()
    last_print_time = start_time
    
    try:
        while True:
            # Receive data package
            state = con.receive()
            
            if state is None:
                continue
            
            current_time = time.time()
            
            # Print data at specified interval
            if current_time - last_print_time >= print_interval:
                print(f"\nTimestamp: {state.timestamp:.3f} s")
                print("-" * 80)
                
                # Print joint positions
                if hasattr(state, 'target_q'):
                    print("Target Joint Positions (target_q):")
                    for i, q in enumerate(state.target_q):
                        print(f"  Joint {i}: {q:.5f} rad ({q * 180 / 3.14159:.2f} deg)")
                
                if hasattr(state, 'actual_q'):
                    print("Actual Joint Positions (actual_q):")
                    for i, q in enumerate(state.actual_q):
                        print(f"  Joint {i}: {q:.5f} rad ({q * 180 / 3.14159:.2f} deg)")
                
                # Print joint velocities
                if hasattr(state, 'target_qd'):
                    print("Target Joint Velocities (target_qd):")
                    for i, qd in enumerate(state.target_qd):
                        print(f"  Joint {i}: {qd:.5f} rad/s")
                
                if hasattr(state, 'actual_qd'):
                    print("Actual Joint Velocities (actual_qd):")
                    for i, qd in enumerate(state.actual_qd):
                        print(f"  Joint {i}: {qd:.5f} rad/s")
                
                # Print TCP pose
                if hasattr(state, 'target_TCP_pose'):
                    print("Target TCP Pose (target_TCP_pose):")
                    print(f"  Position [m]: x={state.target_TCP_pose[0]:.5f}, "
                          f"y={state.target_TCP_pose[1]:.5f}, "
                          f"z={state.target_TCP_pose[2]:.5f}")
                    print(f"  Rotation [rad]: rx={state.target_TCP_pose[3]:.5f}, "
                          f"ry={state.target_TCP_pose[4]:.5f}, "
                          f"rz={state.target_TCP_pose[5]:.5f}")
                
                if hasattr(state, 'actual_TCP_pose'):
                    print("Actual TCP Pose (actual_TCP_pose):")
                    print(f"  Position [m]: x={state.actual_TCP_pose[0]:.5f}, "
                          f"y={state.actual_TCP_pose[1]:.5f}, "
                          f"z={state.actual_TCP_pose[2]:.5f}")
                    print(f"  Rotation [rad]: rx={state.actual_TCP_pose[3]:.5f}, "
                          f"ry={state.actual_TCP_pose[4]:.5f}, "
                          f"rz={state.actual_TCP_pose[5]:.5f}")
                
                # Print TCP speed
                if hasattr(state, 'target_TCP_speed'):
                    print("Target TCP Speed (target_TCP_speed):")
                    tcp_linear_speed = (state.target_TCP_speed[0]**2 + 
                                       state.target_TCP_speed[1]**2 + 
                                       state.target_TCP_speed[2]**2)**0.5
                    print(f"  Linear speed: {tcp_linear_speed:.5f} m/s")
                    print(f"  Angular speed: {state.target_TCP_speed[3]:.5f}, "
                          f"{state.target_TCP_speed[4]:.5f}, "
                          f"{state.target_TCP_speed[5]:.5f} rad/s")
                
                if hasattr(state, 'actual_TCP_speed'):
                    print("Actual TCP Speed (actual_TCP_speed):")
                    tcp_linear_speed = (state.actual_TCP_speed[0]**2 + 
                                       state.actual_TCP_speed[1]**2 + 
                                       state.actual_TCP_speed[2]**2)**0.5
                    print(f"  Linear speed: {tcp_linear_speed:.5f} m/s")
                    print(f"  Angular speed: {state.actual_TCP_speed[3]:.5f}, "
                          f"{state.actual_TCP_speed[4]:.5f}, "
                          f"{state.actual_TCP_speed[5]:.5f} rad/s")
                
                # Print TCP offset
                if hasattr(state, 'tcp_offset'):
                    print("TCP Offset (tcp_offset):")
                    print(f"  Offset [m]: x={state.tcp_offset[0]:.5f}, "
                          f"y={state.tcp_offset[1]:.5f}, "
                          f"z={state.tcp_offset[2]:.5f}")
                    print(f"  Rotation [rad]: rx={state.tcp_offset[3]:.5f}, "
                          f"ry={state.tcp_offset[4]:.5f}, "
                          f"rz={state.tcp_offset[5]:.5f}")
                
                # Print TCP force scalar
                if hasattr(state, 'tcp_force_scalar'):
                    print(f"TCP Force Scalar (tcp_force_scalar): {state.tcp_force_scalar:.2f} N")
                
                # Print tool accelerometer
                if hasattr(state, 'actual_tool_accelerometer'):
                    print("Tool Accelerometer (actual_tool_accelerometer):")
                    print(f"  x={state.actual_tool_accelerometer[0]:.3f}, "
                          f"y={state.actual_tool_accelerometer[1]:.3f}, "
                          f"z={state.actual_tool_accelerometer[2]:.3f} m/s^2")
                
                # Print tool data
                if hasattr(state, 'tool_mode'):
                    print(f"Tool Mode (tool_mode): {state.tool_mode}")
                
                if hasattr(state, 'tool_analog_input0'):
                    print(f"Tool Analog Input 0: {state.tool_analog_input0:.3f}")
                
                if hasattr(state, 'tool_analog_input1'):
                    print(f"Tool Analog Input 1: {state.tool_analog_input1:.3f}")
                
                if hasattr(state, 'tool_output_voltage'):
                    print(f"Tool Output Voltage: {state.tool_output_voltage} V")
                
                if hasattr(state, 'tool_output_current'):
                    print(f"Tool Output Current: {state.tool_output_current:.3f} mA")
                
                if hasattr(state, 'tool_temperature'):
                    print(f"Tool Temperature: {state.tool_temperature:.1f} C")
                
                # Print joint accelerations
                if hasattr(state, 'target_qdd'):
                    print("Target Joint Accelerations (target_qdd):")
                    for i, qdd in enumerate(state.target_qdd):
                        print(f"  Joint {i}: {qdd:.3f} rad/s^2")
                
                # Print joint currents
                if hasattr(state, 'target_current'):
                    print("Target Joint Currents (target_current):")
                    for i, current in enumerate(state.target_current):
                        print(f"  Joint {i}: {current:.3f} A")
                
                if hasattr(state, 'actual_current'):
                    print("Actual Joint Currents (actual_current):")
                    for i, current in enumerate(state.actual_current):
                        print(f"  Joint {i}: {current:.3f} A")
                
                # Print joint moments
                if hasattr(state, 'target_moment'):
                    print("Target Joint Moments (target_moment):")
                    for i, moment in enumerate(state.target_moment):
                        print(f"  Joint {i}: {moment:.3f} Nm")
                
                # Print joint control output
                if hasattr(state, 'joint_control_output'):
                    print("Joint Control Output (joint_control_output):")
                    for i, output in enumerate(state.joint_control_output):
                        print(f"  Joint {i}: {output:.3f}")
                
                # Print joint modes
                if hasattr(state, 'joint_mode'):
                    print("Joint Modes (joint_mode):")
                    for i, mode in enumerate(state.joint_mode):
                        print(f"  Joint {i}: {mode}")
                
                # Print joint voltages
                if hasattr(state, 'actual_joint_voltage'):
                    print("Joint Voltages (actual_joint_voltage):")
                    for i, voltage in enumerate(state.actual_joint_voltage):
                        print(f"  Joint {i}: {voltage:.2f} V")
                
                # Print robot mode and safety status
                if hasattr(state, 'robot_mode'):
                    robot_modes = {
                        -1: "NO_CONTROLLER",
                        0: "DISCONNECTED",
                        1: "CONFIRM_SAFETY",
                        2: "BOOTING",
                        3: "POWER_OFF",
                        4: "POWER_ON",
                        5: "IDLE",
                        6: "BACKDRIVE",
                        7: "RUNNING",
                        8: "UPDATING_FIRMWARE"
                    }
                    mode_str = robot_modes.get(state.robot_mode, f"Unknown Mode({state.robot_mode})")
                    print(f"Robot Mode (robot_mode): {state.robot_mode} - {mode_str}")
                
                if hasattr(state, 'safety_mode'):
                    safety_modes = {
                        1: "Normal",
                        2: "Reduced",
                        3: "Protective Stop",
                        4: "Recovery",
                        5: "Safeguard Stop",
                        6: "System Emergency Stop",
                        7: "Robot Emergency Stop",
                        8: "Emergency Stop",
                        9: "Violation",
                        10: "Fault",
                        11: "Validate Stop"
                    }
                    safety_str = safety_modes.get(state.safety_mode, f"Unknown({state.safety_mode})")
                    print(f"Safety Mode (safety_mode): {safety_str}")
                
                # Print speed scaling
                if hasattr(state, 'speed_scaling'):
                    print(f"Speed Scaling (speed_scaling): {state.speed_scaling * 100:.1f}%")
                
                if hasattr(state, 'target_speed_fraction'):
                    print(f"Target Speed Fraction (target_speed_fraction): {state.target_speed_fraction * 100:.1f}%")
                
                # Print robot status bits
                if hasattr(state, 'robot_status_bits'):
                    print(f"Robot Status Bits (robot_status_bits): {bin(state.robot_status_bits)}")
                    print(f"  Power On: {bool(state.robot_status_bits & 0x01)}")
                    print(f"  Program Running: {bool(state.robot_status_bits & 0x02)}")
                    print(f"  Teach Button Pressed: {bool(state.robot_status_bits & 0x04)}")
                    print(f"  Power Button Pressed: {bool(state.robot_status_bits & 0x08)}")
                
                # Print safety status bits
                if hasattr(state, 'safety_status_bits'):
                    print(f"Safety Status Bits (safety_status_bits): {bin(state.safety_status_bits)}")
                    print(f"  Normal Mode: {bool(state.safety_status_bits & 0x01)}")
                    print(f"  Reduced Mode: {bool(state.safety_status_bits & 0x02)}")
                    print(f"  Protective Stop: {bool(state.safety_status_bits & 0x04)}")
                    print(f"  Recovery Mode: {bool(state.safety_status_bits & 0x08)}")
                    print(f"  Safeguard Stop: {bool(state.safety_status_bits & 0x10)}")
                    print(f"  System Emergency Stop: {bool(state.safety_status_bits & 0x20)}")
                    print(f"  Robot Emergency Stop: {bool(state.safety_status_bits & 0x40)}")
                    print(f"  Emergency Stop: {bool(state.safety_status_bits & 0x80)}")
                
                # Print runtime state
                if hasattr(state, 'runtime_state'):
                    runtime_states = {
                        0: "Stopping",
                        1: "Stopped",
                        2: "Playing",
                        3: "Pausing",
                        4: "Paused",
                        5: "Resuming"
                    }
                    state_str = runtime_states.get(state.runtime_state, f"Unknown({state.runtime_state})")
                    print(f"Runtime State (runtime_state): {state_str}")
                
                # Print digital I/O
                if hasattr(state, 'actual_digital_input_bits'):
                    print(f"Digital Input Bits (actual_digital_input_bits): {bin(state.actual_digital_input_bits)}")
                
                if hasattr(state, 'actual_digital_output_bits'):
                    print(f"Digital Output Bits (actual_digital_output_bits): {bin(state.actual_digital_output_bits)}")
                
                # Print analog I/O
                if hasattr(state, 'standard_analog_input0'):
                    print(f"Standard Analog Input 0: {state.standard_analog_input0:.3f} mA/V")
                
                if hasattr(state, 'standard_analog_input1'):
                    print(f"Standard Analog Input 1: {state.standard_analog_input1:.3f} mA/V")
                
                if hasattr(state, 'standard_analog_output0'):
                    print(f"Standard Analog Output 0: {state.standard_analog_output0:.3f} mA/V")
                
                if hasattr(state, 'standard_analog_output1'):
                    print(f"Standard Analog Output 1: {state.standard_analog_output1:.3f} mA/V")
                
                if hasattr(state, 'analog_io_types'):
                    print(f"Analog I/O Types (analog_io_types): {bin(state.analog_io_types)}")
                
                if hasattr(state, 'io_current'):
                    print(f"I/O Current (io_current): {state.io_current:.3f} mA")
                
                # Print joint temperatures
                if hasattr(state, 'joint_temperatures'):
                    print("Joint Temperatures (joint_temperatures):")
                    for i, temp in enumerate(state.joint_temperatures):
                        print(f"  Joint {i}: {temp:.1f} C")
                
                # Print power data
                if hasattr(state, 'actual_main_voltage'):
                    print(f"Main Voltage (actual_main_voltage): {state.actual_main_voltage:.2f} V")
                
                if hasattr(state, 'actual_robot_voltage'):
                    print(f"Robot Voltage (actual_robot_voltage): {state.actual_robot_voltage:.2f} V")
                
                if hasattr(state, 'actual_robot_current'):
                    print(f"Robot Current (actual_robot_current): {state.actual_robot_current:.3f} A")
                
                # Print elbow data
                if hasattr(state, 'elbow_position'):
                    print("Elbow Position (elbow_position):")
                    print(f"  x={state.elbow_position[0]:.5f}, "
                          f"y={state.elbow_position[1]:.5f}, "
                          f"z={state.elbow_position[2]:.5f} m")
                
                if hasattr(state, 'elbow_velocity'):
                    print("Elbow Velocity (elbow_velocity):")
                    print(f"  x={state.elbow_velocity[0]:.5f}, "
                          f"y={state.elbow_velocity[1]:.5f}, "
                          f"z={state.elbow_velocity[2]:.5f} m/s")
                
                # Print payload data
                if hasattr(state, 'payload'):
                    print(f"Payload Mass (payload): {state.payload:.3f} kg")
                
                if hasattr(state, 'payload_cog'):
                    print("Payload Center of Gravity (payload_cog):")
                    print(f"  x={state.payload_cog[0]:.5f}, "
                          f"y={state.payload_cog[1]:.5f}, "
                          f"z={state.payload_cog[2]:.5f} m")
                
                # Print momentum
                if hasattr(state, 'actual_momentum'):
                    print(f"Actual Momentum (actual_momentum): {state.actual_momentum:.5f}")
                
                # Print execution time
                if hasattr(state, 'actual_execution_time'):
                    print(f"Actual Execution Time: {state.actual_execution_time:.3f} ms")
                
                if hasattr(state, 'target_execution_time'):
                    print(f"Target Execution Time: {state.target_execution_time:.3f} ms")
                
                # Print script control line
                if hasattr(state, 'script_control_line'):
                    print(f"Script Control Line (script_control_line): {state.script_control_line}")
                
                # Print force sensor data
                if hasattr(state, 'actual_TCP_force'):
                    print("TCP Force/Torque (actual_TCP_force):")
                    print(f"  Force [N]: Fx={state.actual_TCP_force[0]:.2f}, "
                          f"Fy={state.actual_TCP_force[1]:.2f}, "
                          f"Fz={state.actual_TCP_force[2]:.2f}")
                    print(f"  Torque [Nm]: Mx={state.actual_TCP_force[3]:.2f}, "
                          f"My={state.actual_TCP_force[4]:.2f}, "
                          f"Mz={state.actual_TCP_force[5]:.2f}")
                
                last_print_time = current_time
                
    except KeyboardInterrupt:
        print("\n\nUser interrupted, stopping monitoring...")
    
    # Stop data synchronization
    con.send_pause()
    print("\nData synchronization stopped")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="RTDE Robot Data Monitoring Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Monitor default variables for 10 seconds:
    %(prog)s --host 192.168.1.15
  
  Monitor specific variables for 30 seconds:
    %(prog)s --host 192.168.1.15 --duration 30 --variables actual_q,actual_TCP_pose
  
  High frequency monitoring (250 Hz):
    %(prog)s --host 192.168.1.15 --frequency 250 --interval 0.2
"""
    )
    
    parser.add_argument(
        '--host',
        type=str,
        default=DEFAULT_ROBOT_IP,
        help=f'Robot IP address (default: {DEFAULT_ROBOT_IP})'
    )
    
    parser.add_argument(
        '--port',
        type=int,
        default=RTDE_PORT,
        help=f'RTDE port (default: {RTDE_PORT})'
    )
    
    parser.add_argument(
        '--interval',
        type=float,
        default=0.5,
        help='Print interval in seconds (default: 0.5)'
    )
    
    parser.add_argument(
        '--variables',
        type=str,
        default=None,
        help='List of variables to read (comma-separated), defaults to common variables'
    )
    
    parser.add_argument(
        '--list-variables',
        action='store_true',
        help='List all available variable names'
    )
    
    args = parser.parse_args()
    
    # List available variables
    if args.list_variables:
        print("Common RTDE Output Variables:")
        print("\nTime and Timestamp:")
        print("  timestamp - Time elapsed since controller was started [s]")
        
        print("\nJoint Data:")
        print("  actual_q - Actual joint positions [rad]")
        print("  actual_qd - Actual joint velocities [rad/s]")
        print("  actual_current - Actual joint currents [A]")
        print("  target_q - Target joint positions [rad]")
        print("  target_qd - Target joint velocities [rad/s]")
        print("  joint_temperatures - Joint temperatures [C]")
        
        print("\nTCP Data:")
        print("  actual_TCP_pose - Actual TCP pose [m, rad]")
        print("  actual_TCP_speed - Actual TCP speed [m/s, rad/s]")
        print("  actual_TCP_force - TCP force/torque [N, Nm]")
        print("  target_TCP_pose - Target TCP pose [m, rad]")
        print("  target_TCP_speed - Target TCP speed [m/s, rad/s]")
        
        print("\nStatus and Mode:")
        print("  robot_mode - Robot mode")
        print("  safety_mode - Safety mode")
        print("  speed_scaling - Speed scaling factor")
        print("  runtime_state - Program runtime state")
        
        print("\nI/O:")
        print("  actual_digital_input_bits - Digital input bits")
        print("  actual_digital_output_bits - Digital output bits")
        print("  standard_analog_input0 - Standard analog input 0 [mA or V]")
        print("  standard_analog_input1 - Standard analog input 1 [mA or V]")
        print("  standard_analog_output0 - Standard analog output 0 [mA or V]")
        print("  standard_analog_output1 - Standard analog output 1 [mA or V]")
        
        print("\nPower:")
        print("  actual_main_voltage - Main voltage [V]")
        print("  actual_robot_voltage - Robot voltage (48V) [V]")
        print("  actual_robot_current - Robot current [A]")
        
        print("\nFor more variables, see:")
        print("  https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html")
        return
    
    # Define variables to read
    if args.variables:
        variables = [v.strip() for v in args.variables.split(',')]
    else:
        # Default variable list - includes all major RTDE outputs
        variables = [
            # Time
            'timestamp',
            # Joint data
            'target_q',
            'target_qd',
            'target_qdd',
            'target_current',
            'target_moment',
            'actual_q',
            'actual_qd',
            'actual_current',
            'joint_control_output',
            'joint_temperatures',
            'joint_mode',
            'actual_joint_voltage',
            # TCP data
            'actual_TCP_pose',
            'actual_TCP_speed',
            'actual_TCP_force',
            'target_TCP_pose',
            'target_TCP_speed',
            'tcp_offset',
            'tcp_force_scalar',
            # Tool data
            'actual_tool_accelerometer',
            'tool_mode',
            'tool_analog_input0',
            'tool_analog_input1',
            'tool_output_voltage',
            'tool_output_current',
            'tool_temperature',
            # Status and mode
            'robot_mode',
            'safety_mode',
            'robot_status_bits',
            'safety_status_bits',
            'speed_scaling',
            'target_speed_fraction',
            'runtime_state',
            # I/O
            'actual_digital_input_bits',
            'actual_digital_output_bits',
            'standard_analog_input0',
            'standard_analog_input1',
            'standard_analog_output0',
            'standard_analog_output1',
            'analog_io_types',
            'io_current',
            # Power and voltage
            'actual_main_voltage',
            'actual_robot_voltage',
            'actual_robot_current',
            # Elbow
            'elbow_position',
            'elbow_velocity',
            # Payload
            'payload',
            'payload_cog',
            # Other
            'actual_momentum',
            'actual_execution_time',
            'target_execution_time',
            'script_control_line',
        ]
    
    try:
        # Connect RTDE
        con = connect_rtde(args.host, args.port)
        
        # Setup output data
        setup_output_recipe(con, variables)
        
        # Monitor data
        monitor_robot_data(con, args.interval)
        
        # Disconnect
        con.disconnect()
        print("\nRTDE connection closed")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
