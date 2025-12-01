from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys
import os

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'common'))

# Auto-detect serial port: prefer description containing 485/RS485; else first /dev/ttyUSB*; else fallback /dev/ttyUSB0
def _auto_select_port(context):
    requested = LaunchConfiguration('modbus_port').perform(context)
    if requested and requested != 'auto':
        print(f"[modbus_manager_launch] Using user specified port: {requested}")
        return requested
    try:
        import serial.tools.list_ports as lp
        ports = list(lp.comports())
        # Filter USB serial-type devices
        usb_ports = [p for p in ports if 'USB' in (p.description or '') or p.device.startswith('/dev/ttyUSB')]
        # Match by keyword 485 / rs485
        for p in usb_ports:
            desc_low = (p.description or '').lower()
            if '485' in desc_low or 'rs485' in desc_low:
                print(f"[modbus_manager_launch] Detected RS485 device: {p.device} ({p.description})")
                return p.device
        # If no keyword match, pick first USB serial
        if usb_ports:
            print(f"[modbus_manager_launch] No 485 keyword found, using first USB serial: {usb_ports[0].device} ({usb_ports[0].description})")
            return usb_ports[0].device
        # Fallback when nothing found
        print("[modbus_manager_launch] No USB serial ports found, fallback to /dev/ttyUSB0")
        return '/dev/ttyUSB0'
    except Exception as e:
        print(f"[modbus_manager_launch] Auto-detect port error: {e}; fallback /dev/ttyUSB0")
        return '/dev/ttyUSB0'

def _launch_setup(context, *args, **kwargs):
    # Try to load from config file
    port = None
    baudrate = None
    enable_dashboard = None
    dashboard_host = None
    dashboard_port = None
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        
        if config.has('lift_robot.modbus_driver.port'):
            port = config.get('lift_robot.modbus_driver.port', 'auto')
            baudrate = config.get('lift_robot.modbus_driver.baudrate', 115200)
            enable_dashboard = config.get('lift_robot.modbus_driver.dashboard.enabled', False)
            dashboard_host = config.get('lift_robot.modbus_driver.dashboard.host', '0.0.0.0')
            dashboard_port = config.get('lift_robot.modbus_driver.dashboard.port', 5000)
            print(f"[modbus_manager_launch] Loaded config: port={port}, baudrate={baudrate}, dashboard={enable_dashboard}")
    except Exception as e:
        print(f"[modbus_manager_launch] Could not load config: {e}")
        print(f"[modbus_manager_launch] Using launch arguments")
    
    # Override with launch arguments if provided
    if port is None:
        port = LaunchConfiguration('modbus_port').perform(context)
    if baudrate is None:
        baudrate = LaunchConfiguration('baudrate').perform(context)
    if enable_dashboard is None:
        enable_dashboard_str = LaunchConfiguration('enable_dashboard').perform(context)
        enable_dashboard = enable_dashboard_str.lower() in ('true', '1', 'yes')
    if dashboard_host is None:
        dashboard_host = LaunchConfiguration('dashboard_host').perform(context)
    if dashboard_port is None:
        dashboard_port = LaunchConfiguration('dashboard_port').perform(context)
    
    # Auto-detect port if needed
    if port == 'auto':
        port = _auto_select_port(context)
    
    # Create node with resolved port / baudrate
    node_params = [
        {'port': port},
        {'baudrate': int(baudrate)},
        {'enable_dashboard': bool(enable_dashboard)},
        {'dashboard_host': str(dashboard_host)},
        {'dashboard_port': int(dashboard_port)}
    ]
    
    node = Node(
        package='modbus_driver',
        executable='modbus_manager_node',
        name='modbus_manager_node',
        parameters=node_params,
        output='screen'
    )
    return [node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modbus_port', default_value='auto', description='Modbus serial device name; auto enables detection'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='Modbus serial baudrate'),
        DeclareLaunchArgument('enable_dashboard', default_value='false', description='Enable Modbus dashboard web interface'),
        DeclareLaunchArgument('dashboard_host', default_value='0.0.0.0', description='Dashboard host address'),
        DeclareLaunchArgument('dashboard_port', default_value='5000', description='Dashboard port'),
        OpaqueFunction(function=_launch_setup)
    ])
