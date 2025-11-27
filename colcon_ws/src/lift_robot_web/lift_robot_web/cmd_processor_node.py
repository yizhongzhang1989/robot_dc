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
        
        # Retry state for current command with timeout detection
        self.current_command = None
        self.current_goal_future = None
        self.retry_count = 0
        self.max_retries = 5  # Increased to handle timeout retries
        self.retry_timer = None
        self.command_lock = threading.Lock()
        
        # Goal response state (updated by async callback)
        self.goal_response = None  # None | 'accepted' | 'rejected' | 'timeout'
        self.goal_send_time = None
        self.timeout_threshold = 0.1  # 100ms timeout
        
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
        """Poll queue and send commands (only if not waiting for retry)"""
        with self.command_lock:
            # Skip if we're waiting for retry or current command response
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
        
        # Start timeout detection loop (only for Action commands)
        if cmd != 'reset':
            self._start_timeout_detection()
    
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
        """Handle goal acceptance/rejection - updates goal_response state"""
        try:
            goal_handle = future.result()
            
            with self.command_lock:
                if goal_handle.accepted:
                    self.goal_response = 'accepted'
                    self.get_logger().info('✅ Goal accepted')
                else:
                    self.goal_response = 'rejected'
                    self.get_logger().warn('⚠️ Goal rejected')
                
        except Exception as e:
            self.get_logger().error(f'Goal response error: {e}')
            with self.command_lock:
                self.goal_response = 'rejected'
    
    def _start_timeout_detection(self):
        """Start timeout detection loop - checks every 100ms for response or timeout"""
        if self.retry_timer:
            self.retry_timer.cancel()
        
        self.retry_timer = self.create_timer(0.1, self._check_response_or_timeout)
    
    def _check_response_or_timeout(self):
        """Check if goal response received or timeout - decides whether to retry"""
        with self.command_lock:
            # If command already cleared (success/reset), stop checking
            if self.current_command is None:
                if self.retry_timer:
                    self.retry_timer.cancel()
                    self.retry_timer = None
                return
            
            # Check response state
            if self.goal_response == 'accepted':
                # ✅ Success - clear and move to next
                self.get_logger().info('✅ Command successful - clearing state')
                self.current_command = None
                self.retry_count = 0
                self.goal_response = None
                if self.retry_timer:
                    self.retry_timer.cancel()
                    self.retry_timer = None
                return
            
            elif self.goal_response == 'rejected':
                # ⚠️ Rejected - retry
                if self.retry_timer:
                    self.retry_timer.cancel()
                    self.retry_timer = None
                self._handle_rejection('rejected')
                return
            
            else:  # goal_response is None
                # Check timeout
                elapsed = time.time() - self.goal_send_time
                if elapsed >= self.timeout_threshold:
                    # ⏱️ Timeout - retry
                    self.get_logger().warn(f'⏱️ Timeout ({elapsed*1000:.0f}ms) - no response received')
                    if self.retry_timer:
                        self.retry_timer.cancel()
                        self.retry_timer = None
                    self._handle_rejection('timeout')
                    return
                # else: still waiting, next timer tick will check again
    
    def _handle_rejection(self, reason):
        """Handle goal rejection/timeout with retry logic"""
        with self.command_lock:
            self.retry_count += 1
            
            if self.retry_count > self.max_retries:
                self.get_logger().error(
                    f'❌ Max retries ({self.max_retries}) exceeded (reason={reason}) - triggering RESET'
                )
                
                # Clear command and trigger emergency reset
                self.current_command = None
                self.retry_count = 0
                self.current_goal_future = None
                self.goal_response = None
                
                # Trigger reset
                self._send_reset()
            else:
                # Retry immediately for rejected, or after checking for timeout
                reason_str = '⏱️ timeout' if reason == 'timeout' else '⚠️ rejected'
                self.get_logger().warn(
                    f'{reason_str} - retrying immediately ({self.retry_count}/{self.max_retries})'
                )
                command = self.current_command
                
                # Retry immediately (no delay timer)
                self._send_command(command, is_retry=True)


def main(args=None):
    rclpy.init(args=args)
    node = CmdProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
