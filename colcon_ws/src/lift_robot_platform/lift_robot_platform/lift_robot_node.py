#!/usr/bin/env python3
"""
Lift Robot Platform ROS2 Node
Controls the lift using relay pulse (flash) commands.
Includes high-frequency closed-loop control for smooth height tracking.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .lift_robot_controller import LiftRobotController
import json
import uuid
import logging

# Configure root logging level
logging.basicConfig(level=logging.INFO)

# ═══════════════════════════════════════════════════════════════
# Control Loop Parameters
# ═══════════════════════════════════════════════════════════════
CONTROL_RATE = 0.1              # Control loop runs at 10 Hz (every 0.1s)
COMMAND_INTERVAL = 1.0          # Coarse control: 1.0s interval
POSITION_TOLERANCE = 0.1        # Target reached within ±0.1mm (high precision) *actually 3.0
CHANGE_THRESHOLD = 0.5          # Send command if target changed by >0.5mm
MAX_STEP = 10.0                 # Limit each position step to ±10mm
LONG_ERROR_THRESHOLD = 20.0     # Errors >20mm = far from target
APPROACH_THRESHOLD = 5.0        # Errors <5mm = near target, increase command frequency
FINE_COMMAND_INTERVAL = 0.2     # Fine control: 0.2s interval (5Hz)
PLATFORM_VELOCITY = 15.0        # Platform velocity: ~15mm/s (constant, hardware-defined)
STOPPING_DISTANCE = 5.0         # Pre-stop distance: compensate for sensor+comm delay (~0.3s × 15mm/s)
CONTROL_ENABLED = True          # Master enable for closed-loop control


class LiftRobotNode(Node):
    def __init__(self):
        super().__init__('lift_robot_platform')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('use_ack_patch', True)
        
        # Retrieve parameters
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        
        # NOTE: Serial port and baudrate are now centrally managed by the modbus_driver node.
        # This node no longer opens the serial device directly; parameters were removed to avoid confusion.
        self.get_logger().info(
            f"Initialize lift platform controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )
        
        # ═══════════════════════════════════════════════════════════════
        # Control Loop State Variables
        # ═══════════════════════════════════════════════════════════════
        self.current_height = 0.0           # Current height from sensor (mm)
        self.target_height = 0.0            # Target height setpoint (mm)
        self.last_sent_height = 0.0         # Last commanded height (mm)
        self.last_command_time = self.get_clock().now()  # Time of last Modbus command
        self.control_enabled = False        # Enable/disable closed-loop control
        self.control_mode = 'manual'        # 'manual' or 'auto' (height control)
        
        # Create controller
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Subscribe to command topic
        self.subscription = self.create_subscription(
            String,
            'lift_robot_platform/command',
            self.command_callback,
            10
        )
        
        # Subscribe to draw-wire sensor for closed-loop height control
        self.sensor_subscription = self.create_subscription(
            String,
            '/draw_wire_sensor/data',
            self.sensor_callback,
            10
        )
        
        # Publish status topic
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_platform/status',
            10
        )
        
        # Status publish timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # High-frequency control loop timer
        self.control_timer = self.create_timer(CONTROL_RATE, self.control_loop)
        
        # Initialize lift platform
        self.controller.initialize()
        
        self.get_logger().info("Lift platform control node started")

    def command_callback(self, msg):
        """Handle command message"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            # Convert seq_id string -> bounded int using hash for uniqueness
            seq_id = abs(hash(seq_id_str)) % 65536  # constrain to 0-65535
            
            self.get_logger().info(f"Received command: {command} [SEQ {seq_id_str}]")
            
            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
                # Also disable auto control if active
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual stop - auto control disabled")
                
            elif command == 'up':
                self.controller.up(seq_id=seq_id)
                
            elif command == 'down':
                self.controller.down(seq_id=seq_id)
                
            elif command == 'timed_up':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
                
            elif command == 'timed_down':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
                
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                
            elif command == 'goto_height':
                # New command: go to specific height with closed-loop control
                target = command_data.get('target_height')
                if target is not None:
                    self.target_height = float(target)
                    self.control_mode = 'auto'
                    self.control_enabled = True
                    # Reset tracking to allow immediate first command
                    self.last_sent_height = self.current_height
                    self.last_command_time = self.get_clock().now() - rclpy.duration.Duration(seconds=COMMAND_INTERVAL)
                    self.get_logger().info(f"[SEQ {seq_id_str}] Auto mode: target height = {self.target_height:.2f} mm")
                else:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] goto_height requires target_height field")
                
            else:
                self.get_logger().warning(f"Unknown command: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling command: {e}")

    def sensor_callback(self, msg):
        """Handle draw-wire sensor feedback for closed-loop control"""
        try:
            sensor_data = json.loads(msg.data)
            # Use adjusted height (includes pushrod offset)
            height = sensor_data.get('height')
            if height is not None:
                self.current_height = float(height)
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            self.get_logger().debug(f"Failed to parse sensor data: {e}")

    def control_loop(self):
        """
        High-frequency control loop (10 Hz) with adaptive precision control.
        
        Three-stage control strategy:
        1. Far (>20mm): Coarse control, 1Hz commands, allow coasting
        2. Approaching (5-20mm): Medium control, 3Hz commands
        3. Fine (<5mm): Precise control, 3Hz commands, small steps
        
        Target precision: ±0.1mm
        """
        if not self.control_enabled or self.control_mode != 'auto':
            return
            
        # Calculate position error and timing
        now = self.get_clock().now()
        dt = (now - self.last_command_time).nanoseconds * 1e-9
        error = self.target_height - self.current_height
        abs_error = abs(error)
        
        # Priority 1: If within tolerance, STOP (only if enough time passed)
        if abs_error <= POSITION_TOLERANCE:
            if dt >= FINE_COMMAND_INTERVAL and self.control_enabled:
                self.controller.stop()
                self.control_enabled = False
                self.get_logger().info(
                    f"[Control] ✅ TARGET REACHED: current={self.current_height:.2f}mm, "
                    f"target={self.target_height:.2f}mm, error={error:.3f}mm"
                )
                self.last_command_time = now
            return
        
        # Determine control stage and command interval
        if abs_error > LONG_ERROR_THRESHOLD:
            # Stage 1: Far from target - coarse control
            command_interval = COMMAND_INTERVAL  # 1Hz
            max_step = MAX_STEP
            stage = "COARSE"
                    
        elif abs_error > APPROACH_THRESHOLD:
            # Stage 2: Approaching target - medium control
            command_interval = FINE_COMMAND_INTERVAL  # 5Hz
            max_step = 5.0
            stage = "APPROACH"
        else:
            # Stage 3: Near target - fine control with predictive stopping
            command_interval = FINE_COMMAND_INTERVAL  # 5Hz
            max_step = 2.0
            stage = "FINE"
            
            # Predictive stop: if within stopping distance, send stop instead of move
            if abs_error <= STOPPING_DISTANCE:
                self.controller.stop()
                self.control_enabled = False
                self.get_logger().info(
                    f"[Control] 🎯 PREDICTIVE STOP: current={self.current_height:.2f}mm, "
                    f"target={self.target_height:.2f}mm, error={error:.3f}mm"
                )
                self.last_command_time = now
                return

        # Check if platform is moving in correct direction (coasting logic for ALL stages)
        if self.last_sent_height != 0 and abs(self.current_height - self.last_sent_height) > 0.5:
            moving_up = self.current_height > self.last_sent_height
            moving_down = self.current_height < self.last_sent_height
            
            # Skip command if moving in correct direction
            if (error > 0 and moving_up) or (error < 0 and moving_down):
                self.get_logger().debug(
                    f"[Control] 🚀 Coasting {stage}: error={error:.2f}mm, moving correctly"
                )
                return
        
        # Priority 2: Check time interval to throttle commands
        if dt < command_interval:
            return  # Too soon, wait for next interval
        
        # Priority 3: Send movement command (pulse width doesn't affect velocity)
        step = max(-max_step, min(max_step, error))
        
        if step > 0.05:  # Moving up (threshold lowered for precision)
            self.controller.up()  # Pulse width is constant (100ms), velocity is hardware-defined
            self.get_logger().info(
                f"[Control] ⬆️  {stage}: current={self.current_height:.2f}mm, "
                f"target={self.target_height:.2f}mm, error={error:.3f}mm"
            )
        elif step < -0.05:  # Moving down
            self.controller.down()  # Pulse width is constant (100ms), velocity is hardware-defined
            self.get_logger().info(
                f"[Control] ⬇️  {stage}: current={self.current_height:.2f}mm, "
                f"target={self.target_height:.2f}mm, error={error:.3f}mm"
            )
        
        # Update tracking variables
        self.last_sent_height = self.current_height + step
        self.last_command_time = now

    def publish_status(self):
        """Publish periodic status info"""
        status = {
            'node': 'lift_robot_platform',
            'device_id': self.device_id,
            'active_timers': len(self.controller.active_timers),
            'control_enabled': self.control_enabled,
            'control_mode': self.control_mode,
            'current_height': self.current_height,
            'target_height': self.target_height,
            'status': 'online'
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """Cleanup resources"""
        self.get_logger().info("Stopping lift platform control node ...")
        
        # Stop platform & cleanup timers
        self.controller.cleanup()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiftRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
