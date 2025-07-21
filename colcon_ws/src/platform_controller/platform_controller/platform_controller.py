from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *
import threading
import time
from collections import deque

class PlatformController(ModbusDevice):
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        
        # Timer-based movement attributes
        self.active_timers = {}  # Dictionary to store active timers
        self.timer_lock = threading.Lock()  # Lock for thread safety
        
        # Command queue for timed operations
        self.timed_cmd_queue = deque()
        self.waiting_for_timed_ack = False

    def initialize(self):
        pass

    def up(self, move_flag=True, seq_id=None):    # move_flag is True to move up, False to stop
        if move_flag:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [1], seq_id=seq_id)
        else:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)

    def down(self, move_flag=True, seq_id=None):  # move_flag is True to move down, False to stop
        if move_flag:
            self.send(5, 0x0002, [1], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)
        else:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)

    def forward(self, move_flag=True, seq_id=None):  # move_flag is True to move forward, False to stop
        if move_flag:
            self.send(5, 0x0000, [1], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)
        else:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)

    def backward(self, move_flag=True, seq_id=None): # move_flag is True to move backward, False to stop
        if move_flag:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [1], seq_id=seq_id)
        else:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)

    def move_with_timer(self, direction, duration_seconds):
        """
        Move platform in specified direction for a given duration, then stop.
        Uses internal ACK mechanism for command sequencing.
        
        Args:
            direction (str): Movement direction ('up', 'down', 'forward', 'backward')
            duration_seconds (float): Duration to move in seconds
        
        Returns:
            dict: Result with success/error status
        """
        try:
            duration_seconds = float(duration_seconds)
            if duration_seconds <= 0:
                return {"error": "Duration must be positive"}
            
            # Cancel any existing timer for this direction
            with self.timer_lock:
                if direction in self.active_timers:
                    # Properly destroy the old timer
                    old_timer = self.active_timers[direction]
                    old_timer.cancel()
                    try:
                        old_timer.destroy()
                    except:
                        pass  # Timer might already be destroyed
                    del self.active_timers[direction]
                    self.node.get_logger().info(f"Cancelled existing timer for {direction}")
            
            # Start movement
            self.node.get_logger().info(f"Starting {direction} movement for {duration_seconds} seconds")
            
            # Generate unique sequence ID for timed commands
            current_time = time.time()
            seq_id = int(current_time * 1000) % 65536  # Keep within valid range [0, 65535]
            
            # Execute start command directly
            self._execute_direction_command(direction, True, seq_id)
            
            # Create timer to stop movement after duration
            # Store direction and seq_id to avoid closure issues
            timer_data = {
                'direction': direction,
                'duration': duration_seconds,
                'start_time': current_time,
                'executed': False  # Flag to prevent multiple executions
            }
            
            def stop_movement():
                try:
                    # Prevent multiple executions
                    if timer_data['executed']:
                        return
                    timer_data['executed'] = True
                    
                    timer_direction = timer_data['direction']
                    self.node.get_logger().info(f"Timer callback: Stopping {timer_direction} movement after {timer_data['duration']} seconds")
                    
                    # Generate new sequence ID for stop command
                    stop_seq_id = int(time.time() * 1000) % 65536
                    
                    # Execute stop command
                    self._execute_direction_command(timer_direction, False, stop_seq_id)
                    
                    # Remove timer from active timers - use a separate lock to avoid deadlock
                    try:
                        with self.timer_lock:
                            if timer_direction in self.active_timers:
                                # Cancel and destroy the timer
                                timer_to_remove = self.active_timers[timer_direction]
                                timer_to_remove.cancel()
                                try:
                                    timer_to_remove.destroy()
                                except:
                                    pass
                                del self.active_timers[timer_direction]
                                self.node.get_logger().info(f"Timer removed for {timer_direction}")
                    except Exception as lock_e:
                        self.node.get_logger().warn(f"Failed to remove timer lock for {timer_direction}: {lock_e}")
                            
                except Exception as e:
                    self.node.get_logger().error(f"Error in timer callback for {timer_data.get('direction', 'unknown')}: {e}")
            
            # Create and start timer using ROS2 timer
            timer = self.node.create_timer(duration_seconds, stop_movement)
            
            # Store timer reference with lock
            with self.timer_lock:
                self.active_timers[direction] = timer
            
            return {
                "success": True, 
                "message": f"Started {direction} movement for {duration_seconds} seconds",
                "seq_id": seq_id
            }
            
        except ValueError:
            return {"error": "Invalid duration format"}
        except Exception as e:
            self.node.get_logger().error(f"Error in move_with_timer: {e}")
            return {"error": str(e)}

    def stop_timed_movement(self, direction):
        """
        Stop any active timed movement for the specified direction.
        
        Args:
            direction (str): Movement direction to stop
            
        Returns:
            dict: Result with success status
        """
        try:
            with self.timer_lock:
                if direction in self.active_timers:
                    # Properly cancel and destroy timer
                    timer = self.active_timers[direction]
                    timer.cancel()
                    try:
                        timer.destroy()
                    except:
                        pass  # Timer might already be destroyed
                    del self.active_timers[direction]
                    
                    # Generate sequence ID for stop command
                    seq_id = int(time.time() * 1000) % 65536
                    
                    # Execute stop command
                    self._execute_direction_command(direction, False, seq_id)
                    
                    self.node.get_logger().info(f"Stopped timed movement for {direction}")
                    return {"success": True, "message": f"Stopped {direction} movement", "seq_id": seq_id}
                else:
                    return {"success": True, "message": f"No active timer for {direction}"}
                    
        except Exception as e:
            self.node.get_logger().error(f"Error stopping timed movement: {e}")
            return {"error": str(e)}

    def stop_all_timed_movements(self):
        """
        Stop all active timed movements.
        
        Returns:
            dict: Result with success status
        """
        try:
            stopped_directions = []
            
            with self.timer_lock:
                # Cancel and destroy all active timers and collect directions
                for direction, timer in list(self.active_timers.items()):
                    timer.cancel()
                    try:
                        timer.destroy()
                    except:
                        pass  # Timer might already be destroyed
                    stopped_directions.append(direction)
                
                # Clear all timers
                self.active_timers.clear()
            
            # Send stop commands for all directions
            for direction in stopped_directions:
                seq_id = int(time.time() * 1000) % 65536
                self._execute_direction_command(direction, False, seq_id)
                self.node.get_logger().info(f"Stopped timed movement for {direction}")
            
            if stopped_directions:
                return {"success": True, "message": f"Stopped timed movements: {stopped_directions}"}
            else:
                return {"success": True, "message": "No active timed movements to stop"}
                
        except Exception as e:
            self.node.get_logger().error(f"Error stopping all timed movements: {e}")
            return {"error": str(e)}

    def _execute_direction_command(self, direction, move_flag, seq_id):
        """
        Internal method to execute direction commands.
        
        Args:
            direction (str): Movement direction
            move_flag (bool): True to start movement, False to stop
            seq_id (int): Sequence ID for tracking
        """
        if direction == "up":
            self.up(move_flag, seq_id=seq_id)
        elif direction == "down":
            self.down(move_flag, seq_id=seq_id)
        elif direction == "forward":
            self.forward(move_flag, seq_id=seq_id)
        elif direction == "backward":
            self.backward(move_flag, seq_id=seq_id)
        else:
            self.node.get_logger().warn(f"[SEQ {seq_id}] Unknown direction: {direction}")

    def cleanup(self):
        """
        Clean up all resources, especially timers.
        Should be called when the node is being destroyed.
        """
        try:
            with self.timer_lock:
                for direction, timer in list(self.active_timers.items()):
                    try:
                        timer.cancel()
                        timer.destroy()
                    except Exception as e:
                        self.node.get_logger().warn(f"Failed to cleanup timer for {direction}: {e}")
                self.active_timers.clear()
                self.node.get_logger().info("All timers cleaned up successfully")
        except Exception as e:
            self.node.get_logger().error(f"Error during cleanup: {e}")
