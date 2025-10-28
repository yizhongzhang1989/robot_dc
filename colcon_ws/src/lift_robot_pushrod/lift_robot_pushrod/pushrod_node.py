#!/usr/bin/env python3
"""
Lift Robot Pushrod ROS2 Node

Controls the pushrod using discrete relay pulses (relay3 stop, relay4 up, relay5 down).
Each action sends a single-coil ON (0xFF00) followed ~100ms later by OFF (0x0000),
mirroring the lift platform controller style. Timed commands issue a direction pulse
then schedule a stop pulse after the specified duration.

Includes closed-loop height control using filtered sensor readings.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .pushrod_controller import PushrodController
import json
import uuid
import logging

logging.basicConfig(level=logging.INFO)

# ═══════════════════════════════════════════════════════════════
# Control Loop Parameters for Pushrod
# ═══════════════════════════════════════════════════════════════
CONTROL_RATE = 0.02             # Control loop runs at 50 Hz (every 0.02s)
POSITION_TOLERANCE = 0.5        # Target reached within ±0.5mm
COMMAND_INTERVAL = 0.3          # Command send interval: 0.3s (prevent too frequent commands)
HISTORY_SIZE = 4                # Number of height readings to keep for filtering

class PushrodNode(Node):
    def __init__(self):
        super().__init__('lift_robot_pushrod')

        # Parameters
        # Default device_id corrected to 50 (0x32) per hardware mapping
        self.declare_parameter('device_id', 50)
        self.declare_parameter('use_ack_patch', True)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value

        self.get_logger().info(
            f"Initialize pushrod controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )

        # ═══════════════════════════════════════════════════════════════
        # Control Loop State Variables
        # ═══════════════════════════════════════════════════════════════
        self.current_height = 0.0           # Current filtered height (mm)
        self.target_height = 0.0            # Target height setpoint (mm)
        self.height_history = []            # History of height readings for filtering
        self.last_command_time = self.get_clock().now()  # Time of last Modbus command
        self.control_enabled = False        # Enable/disable closed-loop control
        self.control_mode = 'manual'        # 'manual' or 'auto' (height control)
        self.movement_state = 'stop'        # Current movement state: 'up', 'down', or 'stop'
        
        # Pushrod offset tracking
        self.pushrod_offset = 0.0           # Current pushrod offset in mm
        self.height_before_movement = None  # Height before starting movement
        self.is_tracking_offset = False     # Flag to track if we should measure offset

        # Controller
        self.controller = PushrodController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Set callback for auto-stop completion (for offset tracking)
        self.controller.on_auto_stop_callback = self._on_auto_stop_complete

        # Command subscription
        self.subscription = self.create_subscription(
            String,
            'lift_robot_pushrod/command',
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

        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_pushrod/status',
            10
        )

        # Timer for status
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # High-frequency control loop timer
        self.control_timer = self.create_timer(CONTROL_RATE, self.control_loop)

        # Initialize
        self.controller.initialize()

        self.get_logger().info("Pushrod control node started")

    def command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            seq_id = abs(hash(seq_id_str)) % 65536

            self.get_logger().info(f"Received pushrod command: {command} [SEQ {seq_id_str}]")

            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
                # Calculate offset if we were tracking
                if self.is_tracking_offset and self.height_before_movement is not None:
                    height_change = self.current_height - self.height_before_movement
                    self.pushrod_offset += height_change
                    self.get_logger().info(f"[SEQ {seq_id_str}] Pushrod offset updated: {height_change:.2f}mm, total offset: {self.pushrod_offset:.2f}mm")
                    self.is_tracking_offset = False
                    self.height_before_movement = None
                # Also disable auto control if active
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual stop - auto control disabled")
                self.movement_state = 'stop'
                    
            elif command == 'up':
                # Start tracking offset
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.up(seq_id=seq_id)
                self.movement_state = 'up'
                
            elif command == 'down':
                # Start tracking offset
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.down(seq_id=seq_id)
                self.movement_state = 'down'
                
            elif command == 'timed_up':
                # Start tracking offset
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
                
            elif command == 'timed_down':
                # Start tracking offset
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
                
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                
            elif command == 'goto_point':
                point = command_data.get('point')
                if not point:
                    self.get_logger().warning("goto_point command missing 'point' field")
                else:
                    # Reset offset when going to base
                    if point == 'base':
                        self.pushrod_offset = 0.0
                        self.is_tracking_offset = False
                        self.height_before_movement = None
                        self.get_logger().info(f"[SEQ {seq_id_str}] Going to base - pushrod offset reset to 0")
                    else:
                        # For other points, start tracking offset
                        self.height_before_movement = self.current_height
                        self.is_tracking_offset = True
                    self.controller.goto_point(point, seq_id=seq_id)
                    
            elif command == 'goto_height':
                # New command: go to specific height with closed-loop control
                target = command_data.get('target_height')
                if target is not None:
                    # Start tracking offset for fine adjustment
                    self.height_before_movement = self.current_height
                    self.is_tracking_offset = True
                    self.target_height = float(target)
                    self.control_mode = 'auto'
                    self.control_enabled = True
                    # Reset tracking to allow immediate first command
                    self.height_history.clear()
                    self.movement_state = 'stop'  # Reset movement state
                    self.last_command_time = self.get_clock().now() - rclpy.duration.Duration(seconds=COMMAND_INTERVAL)
                    self.get_logger().info(f"[SEQ {seq_id_str}] Pushrod auto mode: target height = {self.target_height:.2f} mm")
                else:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] goto_height requires target_height field")
                    
            else:
                self.get_logger().warning(f"Unknown pushrod command: {command}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling pushrod command: {e}")

    def sensor_callback(self, msg):
        """Handle draw-wire sensor feedback for closed-loop control"""
        try:
            sensor_data = json.loads(msg.data)
            # Use adjusted height (includes pushrod offset)
            height = sensor_data.get('height')
            if height is not None:
                raw_height = float(height)
                
                # Add to history
                self.height_history.append(raw_height)
                
                # Keep only the most recent HISTORY_SIZE readings
                if len(self.height_history) > HISTORY_SIZE:
                    self.height_history.pop(0)
                
                # Calculate filtered height (remove min/max, average the rest)
                self.current_height = self._calculate_filtered_height()
                
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            self.get_logger().debug(f"Failed to parse sensor data: {e}")
    
    def _calculate_filtered_height(self):
        """
        Calculate filtered height by removing min/max values and averaging the rest.
        If less than 4 samples, return the average of available samples.
        """
        if len(self.height_history) == 0:
            return self.current_height  # No new data, keep old value
        
        if len(self.height_history) < 4:
            # Not enough samples, return simple average
            return sum(self.height_history) / len(self.height_history)
        
        # Remove min and max, average the rest
        sorted_heights = sorted(self.height_history)
        middle_values = sorted_heights[1:-1]  # Remove first (min) and last (max)
        return sum(middle_values) / len(middle_values)
    
    def _on_auto_stop_complete(self):
        """Callback when auto-stop timer completes (for offset tracking)."""
        if self.is_tracking_offset and self.height_before_movement is not None:
            height_change = self.current_height - self.height_before_movement
            self.pushrod_offset += height_change
            self.get_logger().info(f"Auto-stop complete. Offset updated: {height_change:.2f}mm, total: {self.pushrod_offset:.2f}mm")
            self.is_tracking_offset = False
            self.height_before_movement = None

    def control_loop(self):
        """
        Simplified closed-loop control with filtered height measurement for pushrod.
        
        Control strategy:
        1. Use filtered height (remove min/max from last 4 readings)
        2. Calculate error = target - current
        3. Send up/down/stop command based on error
        4. Limit command frequency to prevent excessive Modbus traffic
        """
        try:
            if not self.control_enabled or self.control_mode != 'auto':
                return
            
            # Need at least some readings to start control
            if len(self.height_history) == 0:
                return
                
            # Calculate position error
            error = self.target_height - self.current_height
            abs_error = abs(error)
            
            # Calculate time since last command
            now = self.get_clock().now()
            dt = (now - self.last_command_time).nanoseconds * 1e-9
            
        except Exception as e:
            self.get_logger().error(f"[Pushrod Control] Loop error (calculation): {e}")
            return
        
        # Priority 1: Check if target reached
        if abs_error <= POSITION_TOLERANCE:
            if self.control_enabled:
                self.controller.stop()
                # Calculate offset after automatic movement
                if self.is_tracking_offset and self.height_before_movement is not None:
                    height_change = self.current_height - self.height_before_movement
                    self.pushrod_offset += height_change
                    self.get_logger().info(f"[Pushrod Control] Offset updated: {height_change:.2f}mm, total: {self.pushrod_offset:.2f}mm")
                    self.is_tracking_offset = False
                    self.height_before_movement = None
                self.control_enabled = False
                self.movement_state = 'stop'
                self.get_logger().info(
                    f"[Pushrod Control] ✅ TARGET REACHED: current={self.current_height:.2f}mm, "
                    f"target={self.target_height:.2f}mm, error={error:.3f}mm"
                )
                self.last_command_time = now
            return
        
        # Priority 2: Check command interval (throttling)
        if dt < COMMAND_INTERVAL:
            return  # Too soon, wait for next interval
        
        # Priority 3: Send movement command based on error direction
        try:
            if error > POSITION_TOLERANCE:
                # Need to move up - only send command if not already moving up
                if self.movement_state != 'up':
                    self.controller.up()
                    self.movement_state = 'up'
                    self.get_logger().info(
                        f"[Pushrod Control] ⬆️  UP: current={self.current_height:.2f}mm, "
                        f"target={self.target_height:.2f}mm, error={error:.2f}mm"
                    )
                    self.last_command_time = now
                else:
                    self.get_logger().debug(
                        f"[Pushrod Control] ⏫ Already moving UP: current={self.current_height:.2f}mm, "
                        f"target={self.target_height:.2f}mm, error={error:.2f}mm"
                    )
                
            elif error < -POSITION_TOLERANCE:
                # Need to move down - only send command if not already moving down
                if self.movement_state != 'down':
                    self.controller.down()
                    self.movement_state = 'down'
                    self.get_logger().info(
                        f"[Pushrod Control] ⬇️  DOWN: current={self.current_height:.2f}mm, "
                        f"target={self.target_height:.2f}mm, error={error:.2f}mm"
                    )
                    self.last_command_time = now
                else:
                    self.get_logger().debug(
                        f"[Pushrod Control] ⏬ Already moving DOWN: current={self.current_height:.2f}mm, "
                        f"target={self.target_height:.2f}mm, error={error:.2f}mm"
                    )
                
        except Exception as e:
            self.get_logger().error(f"[Pushrod Control] Command execution error: {e}")
            # Don't disable control on command errors, just skip this cycle

    def publish_status(self):
        try:
            status = {
                'node': 'lift_robot_pushrod',
                'device_id': self.device_id,
                'has_stop_timer': bool(self.controller.stop_timer) if hasattr(self.controller, 'stop_timer') else False,
                'current_position_seconds': self.controller.current_position if hasattr(self.controller, 'current_position') else 0.0,
                'current_point': self.controller.current_point if hasattr(self.controller, 'current_point') else 'unknown',
                'control_enabled': self.control_enabled,
                'control_mode': self.control_mode,
                'current_height': self.current_height,
                'target_height': self.target_height,
                'movement_state': self.movement_state,
                'pushrod_offset': self.pushrod_offset,
                'is_tracking_offset': self.is_tracking_offset,
                'status': 'online'
            }
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"Pushrod status publish error: {e}")
            # Continue operation

    def destroy_node(self):
        self.get_logger().info("Stopping pushrod control node ...")
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PushrodNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Pushrod node runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
