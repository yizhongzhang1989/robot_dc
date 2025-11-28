#!/usr/bin/env python3
"""
Command Processor Node - Queue-based command forwarder with retry mechanism.
Polls command queue at 50Hz and forwards to platform with automatic retry on failure.
NO control logic - all logic in platform node.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import json
import threading
import time
from collections import deque


class CmdProcessorNode(Node):
    def __init__(self):
        super().__init__('lift_robot_cmd_processor')
        
        # Command queue (received from web)
        self.command_queue = deque()
        self.queue_lock = threading.Lock()
        
        # Retry state for current command (reject-based retry only)
        self.current_command = None
        self.current_goal_future = None
        self.retry_count = 0
        self.max_retries = 3  # Only retry on explicit rejection
        self.command_lock = threading.Lock()
        
        # Goal response state (updated by async callback)
        self.goal_response = None  # None | 'accepted' | 'rejected'
        self.goal_send_time = None
        self.command_timeout = 1.0  # 1 second timeout to force skip stuck commands
        
        # Platform command publisher (for reset via topic)
        self.platform_cmd_pub = self.create_publisher(
            String,
            '/lift_robot_platform/command',
            10
        )
        
        # Command queue subscription
        self.queue_sub = self.create_subscription(
            String,
            '/lift_robot/command_queue',
            self._queue_callback,
            100
        )
        
        # Action clients
        self.action_clients = {}
        self._setup_action_clients()
        
        # Polling timer - 50Hz for queue processing
        self.poll_timer = self.create_timer(0.02, self._process_queue)
        
        self.get_logger().info('🚀 Cmd Processor ready (retry-enabled, 50Hz)')
    
    def _setup_action_clients(self):
        from lift_robot_interfaces.action import GotoHeight, ForceControl, HybridControl, ManualMove, StopMovement
        
        self.action_clients['goto_height'] = ActionClient(self, GotoHeight, '/lift_robot/goto_height')
        self.action_clients['force_control'] = ActionClient(self, ForceControl, '/lift_robot/force_control')
        self.action_clients['hybrid_control'] = ActionClient(self, HybridControl, '/lift_robot/hybrid_control')
        self.action_clients['manual_move'] = ActionClient(self, ManualMove, '/lift_robot/manual_move')
        self.action_clients['stop_movement'] = ActionClient(self, StopMovement, '/lift_robot/stop_movement')
        
        self.get_logger().info(f'Action clients ready: {list(self.action_clients.keys())}')
    
    def _queue_callback(self, msg):
        """Receive command and add to queue"""
        with self.queue_lock:
            self.command_queue.append(json.loads(msg.data))
    
    def _process_queue(self):
        """Poll queue and send commands (check timeout first)"""
        with self.command_lock:
            # Check if current command timed out (1 second)
            if self.current_command is not None:
                self._check_command_timeout()
            
            # Skip if still waiting for current command
            if self.current_command is not None:
                return
        
        if not self.command_queue:
            return
        
        with self.queue_lock:
            command = self.command_queue.popleft()
        
        self._send_command(command)
    
    def _send_command(self, command, is_retry=False):
        """Send command to platform with timeout detection"""
        with self.command_lock:
            self.current_command = command
            if not is_retry:
                self.retry_count = 0
            # Reset response state for new send
            self.goal_response = None
            self.goal_send_time = time.time()
        
        cmd = command.get('command')
        target = command.get('target', 'platform')
        
        retry_info = f" (retry {self.retry_count}/{self.max_retries})" if is_retry else ""
        self.get_logger().info(f'→ {cmd}/{target}{retry_info}')
        
        # Route commands
        if cmd in ('up', 'down'):
            self._send_manual_move(cmd, target)
        elif cmd == 'stop':
            self._send_stop_movement(target)
        elif cmd == 'reset':
            self._send_reset()
        elif cmd == 'goto_height':
            self._send_goto_height(command, target)
        elif cmd in ('force_up', 'force_down'):
            self._send_force_control(command, cmd)
        elif cmd == 'height_force_hybrid':
            self._send_hybrid(command)
        else:
            # Unknown command, clear and continue
            with self.command_lock:
                self.current_command = None
                self.goal_response = None
    
    def _send_manual_move(self, direction, target):
        from lift_robot_interfaces.action import ManualMove
        goal = ManualMove.Goal()
        goal.direction = direction
        goal.target = target
        
        future = self.action_clients['manual_move'].send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        with self.command_lock:
            self.current_goal_future = future
    
    def _send_stop_movement(self, target):
        """Send StopMovement action (with retry support)"""
        from lift_robot_interfaces.action import StopMovement
        goal = StopMovement.Goal()
        goal.target = target
        
        future = self.action_clients['stop_movement'].send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        with self.command_lock:
            self.current_goal_future = future
    
    def _send_reset(self):
        """Emergency reset via topic"""
        msg = String()
        msg.data = json.dumps({'command': 'reset'})
        self.platform_cmd_pub.publish(msg)
        self.get_logger().warn('🔴 RESET')
        
        # Clear queue and current command
        with self.queue_lock:
            self.command_queue.clear()
        with self.command_lock:
            self.current_command = None
            self.retry_count = 0
    
    def _send_goto_height(self, command, target):
        from lift_robot_interfaces.action import GotoHeight
        goal = GotoHeight.Goal()
        goal.target = target
        goal.target_height = float(command.get('target_height'))
        goal.mode = command.get('mode', 'absolute')
        
        future = self.action_clients['goto_height'].send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        with self.command_lock:
            self.current_goal_future = future
    
    def _send_force_control(self, command, cmd):
        from lift_robot_interfaces.action import ForceControl
        goal = ForceControl.Goal()
        goal.target_force = float(command.get('target_force'))
        goal.direction = cmd.replace('force_', '')
        
        future = self.action_clients['force_control'].send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        with self.command_lock:
            self.current_goal_future = future
    
    def _send_hybrid(self, command):
        from lift_robot_interfaces.action import HybridControl
        goal = HybridControl.Goal()
        goal.target_height = float(command.get('target_height'))
        goal.target_force = float(command.get('target_force'))
        
        future = self.action_clients['hybrid_control'].send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        
        with self.command_lock:
            self.current_goal_future = future
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection - immediately clear on accept, retry on reject"""
        try:
            goal_handle = future.result()
            
            if goal_handle.accepted:
                # ✅ Accepted - clear command state immediately
                with self.command_lock:
                    self.get_logger().info('✅ Goal accepted - clearing command state')
                    self.current_command = None
                    self.retry_count = 0
                    self.goal_response = None
            else:
                # ⚠️ Rejected - trigger retry
                with self.command_lock:
                    self.goal_response = 'rejected'
                    self.get_logger().warn('⚠️ Goal rejected')
                self._handle_rejection()
                
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            with self.command_lock:
                self.goal_response = 'rejected'
            self._handle_rejection()
    
    def _check_command_timeout(self):
        """Check if command has been waiting too long (1 second) - force skip if stuck"""
        with self.command_lock:
            # If command already cleared, do nothing
            if self.current_command is None:
                return
            
            # Check if command has been waiting for 1 second
            elapsed = time.time() - self.goal_send_time
            if elapsed >= self.command_timeout:
                # Force skip stuck command
                self.get_logger().error(
                    f'⏱️ Command TIMEOUT ({elapsed:.1f}s) - no callback received, skipping to next command'
                )
                self.current_command = None
                self.retry_count = 0
                self.goal_response = None
    
    def _handle_rejection(self):
        """Handle goal rejection with retry logic (reject callback only)"""
        with self.command_lock:
            self.retry_count += 1
            
            if self.retry_count > self.max_retries:
                self.get_logger().error(
                    f'❌ Max retries ({self.max_retries}) exceeded - skipping command'
                )
                
                # Skip command without reset
                self.current_command = None
                self.retry_count = 0
                self.current_goal_future = None
                self.goal_response = None
            else:
                # Retry immediately on rejection
                self.get_logger().warn(
                    f'⚠️ Goal rejected - retrying ({self.retry_count}/{self.max_retries})'
                )
                command = self.current_command
                
                # Retry immediately
                self._send_command(command, is_retry=True)


def main(args=None):
    rclpy.init(args=args)
    node = CmdProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
