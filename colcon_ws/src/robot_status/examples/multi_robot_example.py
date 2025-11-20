#!/usr/bin/env python3
"""
Example: Multi-robot simulation

Simulates multiple robots updating their status continuously.
This demonstrates dynamic robot addition and real-time updates.
"""

import rclpy
from rclpy.node import Node
from robot_status.client_utils import RobotStatusClient
import time
import random
import math


class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # Create status client
        self.client = RobotStatusClient(self)
        
        # Robot configuration
        self.num_robots = 3
        self.update_rate = 1.0  # Hz
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_status)
        
        self.get_logger().info(f"Simulating {self.num_robots} robots")
        self.get_logger().info(f"View updates at: http://localhost:8005")
        
        # Initialize robot positions
        self.positions = {}
        for i in range(1, self.num_robots + 1):
            self.positions[f'robot{i}'] = {
                'x': random.uniform(-5, 5),
                'y': random.uniform(-5, 5),
                'z': 0.0,
                'theta': random.uniform(0, 360)
            }
    
    def update_status(self):
        """Update status for all simulated robots."""
        
        # Update shared status
        active_robots = [f'robot{i}' for i in range(1, self.num_robots + 1)]
        self.client.set_status('shared', 'active_robots', active_robots)
        self.client.set_status('shared', 'timestamp', time.time())
        self.client.set_status('shared', 'total_tasks_completed', random.randint(100, 200))
        
        # Update each robot
        for i in range(1, self.num_robots + 1):
            robot_name = f'robot{i}'
            
            # Update position (random walk)
            pos = self.positions[robot_name]
            pos['x'] += random.uniform(-0.2, 0.2)
            pos['y'] += random.uniform(-0.2, 0.2)
            pos['theta'] = (pos['theta'] + random.uniform(-10, 10)) % 360
            
            self.client.set_status(robot_name, 'pose', pos)
            
            # Update battery (slowly decreasing)
            current_battery = self.client.get_status(robot_name, 'battery')
            if current_battery is None:
                current_battery = 100
            new_battery = max(0, current_battery - random.uniform(0, 0.5))
            self.client.set_status(robot_name, 'battery', round(new_battery, 1))
            
            # Update gripper state (random)
            gripper_states = ['open', 'closed', 'grasping', 'idle']
            self.client.set_status(robot_name, 'gripper_state', random.choice(gripper_states))
            
            # Update mode
            modes = ['autonomous', 'manual', 'charging', 'idle']
            if new_battery < 20:
                mode = 'charging'
            else:
                mode = random.choice(modes)
            self.client.set_status(robot_name, 'mode', mode)
            
            # Update velocity
            velocity = {
                'linear': round(random.uniform(0, 1.5), 2),
                'angular': round(random.uniform(-0.5, 0.5), 2)
            }
            self.client.set_status(robot_name, 'velocity', velocity)
            
            # Update task
            task_types = ['pick', 'place', 'navigate', 'idle']
            self.client.set_status(robot_name, 'current_task', {
                'type': random.choice(task_types),
                'progress': round(random.uniform(0, 100), 1)
            })
        
        self.get_logger().info(f"Updated status for {self.num_robots} robots")


def main():
    rclpy.init()
    
    simulator = None
    try:
        simulator = RobotSimulator()
        rclpy.spin(simulator)
        
    except KeyboardInterrupt:
        print("\nShutting down simulator...")
    finally:
        if simulator:
            simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
