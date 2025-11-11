#!/usr/bin/env python3
"""
Lift Robot Server Positioning Script

Task:
    Phase 1 - Initialization:
        1. Platform DOWN for 15 seconds
        2. Pushrod DOWN for 7 seconds
        3. Read current height after both stopped
        4. Use pushrod goto_height to adjust to (current_height + 15mm)
    
    Phase 2 - Raise to Server:
        1. Platform UP continuously
        2. Monitor force sensor (raw value, no filtering)
        3. Stop when force >= 750 N (default threshold)

Produces JSON summary on stdout:
{
    "initial_height": <float>,
    "adjusted_height": <float>,
    "final_height": <float>,
    "final_force": <float>,
    "force_threshold": <float>,
    "status": "success"|"error",
    "error": <str or null>
}

Run after sourcing ROS2 workspace:
    source install/setup.bash
    python3 scripts/lift_robot_server_positioning.py

Dry run (no motion, simulates readings):
    python3 scripts/lift_robot_server_positioning.py --dry-run

Topics used:
    - /lift_robot_platform/command (String JSON): {"command": "down"/"up"/"stop"}
    - /lift_robot_pushrod/command (String JSON): {"command": "down"/"stop", "goto_height": target}
    - /draw_wire_sensor/data (String JSON): {"height": <mm>}
    - /force_sensor (Float32): raw force value in Newtons
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json, time, sys, argparse, math

class PositioningNode(Node):
    def __init__(self, args):
        super().__init__('lift_robot_server_positioning')
        self.args = args
        
        # Publisher for platform commands
        self.platform_cmd_pub = self.create_publisher(String, '/lift_robot_platform/command', 10)
        
        # Publisher for pushrod commands
        self.pushrod_cmd_pub = self.create_publisher(String, '/lift_robot_pushrod/command', 10)
        
        # Height sensor subscription
        self.height = None
        self.height_sub = self.create_subscription(String, '/draw_wire_sensor/data', self.height_cb, 10)
        
        # Force sensor subscription (raw value, no filtering)
        self.force = None
        self.force_sub = self.create_subscription(Float32, '/force_sensor', self.force_cb, 10)
        
        # Results
        self.initial_height = None      # Height after platform+pushrod down
        self.adjusted_height = None     # Target height for pushrod adjustment (initial + 15mm)
        self.final_height = None        # Height when force threshold reached
        self.final_force = None         # Force when stopped

    def height_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            h = data.get('height')
            if h is not None:
                self.height = float(h)
        except Exception as e:
            self.get_logger().warn(f"Height parse error: {e}")

    def force_cb(self, msg: Float32):
        """Store raw force value (no filtering applied)"""
        try:
            self.force = float(msg.data)
            # Track force sampling timestamps for frequency calculation
            if not hasattr(self, 'force_timestamps'):
                self.force_timestamps = []
            self.force_timestamps.append(time.time())
            # Keep only last 100 timestamps for frequency calculation
            if len(self.force_timestamps) > 100:
                self.force_timestamps.pop(0)
        except Exception as e:
            self.get_logger().warn(f"Force parse error: {e}")
    
    def get_force_sampling_rate(self):
        """Calculate force sensor sampling rate in Hz"""
        if not hasattr(self, 'force_timestamps') or len(self.force_timestamps) < 2:
            return None
        timestamps = self.force_timestamps
        intervals = [timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else None

    def publish_cmd(self, publisher, payload: dict, label: str):
        """Publish command to a topic"""
        if self.args.dry_run:
            self.get_logger().info(f"[DRY-RUN] Would publish to {label}: {payload}")
            return
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)
        self.get_logger().info(f"Published {label} command: {payload}")
        
        # Give some time for command to be processed
        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.0)

    def wait_for_height(self, timeout):
        """Wait for height data to be available"""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.height is not None:
                return True
        return False

    def wait_for_force(self, timeout):
        """Wait for force data to be available"""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.force is not None:
                return True
        return False

    def perform_initialization(self):
        """
        Phase 1: Initialization
        1. Platform DOWN for 15 seconds
        2. Pushrod DOWN for 7 seconds
        3. Read current height after stabilization
        4. Pushrod goto_height to (current_height + 15mm)
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("Phase 1: Initialization")
        self.get_logger().info("=" * 60)
        
        # Step 1: Platform DOWN for 15 seconds
        self.get_logger().info("Step 1: Platform DOWN for 15 seconds...")
        self.publish_cmd(self.platform_cmd_pub, {"command": "down"}, 'platform')
        
        if self.args.dry_run:
            time.sleep(1.0)
        else:
            time.sleep(15.0)
        
        self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
        self.get_logger().info("Platform DOWN complete, stopped.")
        
        # Step 2: Pushrod DOWN for 7 seconds
        self.get_logger().info("Step 2: Pushrod DOWN for 7 seconds...")
        self.publish_cmd(self.pushrod_cmd_pub, {"command": "down"}, 'pushrod')
        
        if self.args.dry_run:
            time.sleep(0.5)
        else:
            time.sleep(7.0)
        
        self.publish_cmd(self.pushrod_cmd_pub, {"command": "stop"}, 'pushrod')
        self.get_logger().info("Pushrod DOWN complete, stopped.")
        
        # Step 3: Wait for height stabilization and read current height
        self.get_logger().info("Step 3: Waiting for height stabilization...")
        time.sleep(2.0)  # Wait for mechanical settling
        
        if not self.wait_for_height(timeout=5.0):
            raise RuntimeError("No height data received after initialization")
        
        # Dry-run simulation
        if self.args.dry_run:
            self.height = 800.0  # Simulated base height
        
        self.initial_height = self.height
        self.get_logger().info(f"Initial height captured: {self.initial_height:.2f} mm")
        
        # Step 4: Pushrod adjustment to (current_height + 15mm)
        self.adjusted_height = self.initial_height + self.args.pushrod_offset_mm
        self.get_logger().info(f"Step 4: Adjusting pushrod to {self.adjusted_height:.2f} mm (offset +{self.args.pushrod_offset_mm} mm)...")
        
        self.publish_cmd(
            self.pushrod_cmd_pub,
            {"command": "goto_height", "target_height": self.adjusted_height},
            'pushrod'
        )
        
        # Wait for pushrod to reach target (with timeout)
        if self.args.dry_run:
            time.sleep(1.0)
            self.height = self.adjusted_height
        else:
            start = time.time()
            reached = False
            while rclpy.ok() and time.time() - start < self.args.pushrod_adjust_timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.height is not None and abs(self.height - self.adjusted_height) <= self.args.height_tolerance:
                    reached = True
                    self.get_logger().info(f"Pushrod reached target: {self.height:.2f} mm")
                    break
            
            if not reached:
                self.get_logger().warn(f"Pushrod adjustment timeout (current: {self.height:.2f} mm, target: {self.adjusted_height:.2f} mm)")
        
        self.get_logger().info(f"Initialization complete: initial={self.initial_height:.2f} mm, adjusted={self.adjusted_height:.2f} mm")
        self.get_logger().info("")

    def raise_to_server(self):
        """
        Phase 2: Raise to Server Position
        1. Platform UP continuously
        2. Monitor raw force sensor (no filtering)
        3. Stop when force >= threshold (default 750 N)
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("Phase 2: Raising platform to server position")
        self.get_logger().info("=" * 60)
        
        thresh = self.args.force_threshold
        self.get_logger().info(f"Monitoring force sensor (raw value, no filtering)")
        self.get_logger().info(f"Stop condition: force >= {thresh:.1f} N")
        
        # Wait for initial force reading
        if not self.wait_for_force(timeout=3.0):
            self.get_logger().warn("No force data available, proceeding anyway...")
        
        # Start continuous UP motion
        self.publish_cmd(self.platform_cmd_pub, {"command": "up"}, 'platform')
        
        start = time.time()
        threshold_reached = False
        loop_period = 0.02  # 50 Hz monitoring
        last_log_time = start
        sample_count = 0
        
        # Dry-run simulation
        if self.args.dry_run:
            time.sleep(2.0)
            self.force = thresh + 10.0
            self.height = self.adjusted_height + 20.0
            threshold_reached = True
            self.get_logger().info(f"[DRY-RUN] Simulated force threshold reached: {self.force:.2f} N")
        else:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=loop_period)
                sample_count += 1
                
                if self.force is not None:
                    # Check raw force value (no filtering)
                    if self.force >= thresh:
                        threshold_reached = True
                        elapsed = time.time() - start
                        sampling_rate = self.get_force_sampling_rate()
                        self.get_logger().info(f"Force threshold reached: {self.force:.2f} N >= {thresh:.2f} N")
                        self.get_logger().info(f"Time elapsed: {elapsed:.2f}s, Loop samples: {sample_count}")
                        if sampling_rate:
                            self.get_logger().info(f"Force sensor sampling rate: {sampling_rate:.1f} Hz")
                        break
                    
                    # Log progress every 2 seconds
                    elapsed = time.time() - start
                    if elapsed - last_log_time >= 2.0:
                        sampling_rate = self.get_force_sampling_rate()
                        rate_str = f"{sampling_rate:.1f} Hz" if sampling_rate else "N/A"
                        self.get_logger().info(
                            f"Progress: force={self.force:.2f} N, height={self.height:.2f} mm, "
                            f"elapsed={elapsed:.1f}s, sampling_rate={rate_str}"
                        )
                        last_log_time = elapsed
        
        # Stop platform
        self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
        
        # Record final values
        self.final_force = self.force
        self.final_height = self.height
        final_sampling_rate = self.get_force_sampling_rate()
        
        self.get_logger().info(f"Raise complete: final_force={self.final_force:.2f} N, final_height={self.final_height:.2f} mm")
        if final_sampling_rate:
            self.get_logger().info(f"Final force sensor sampling rate: {final_sampling_rate:.1f} Hz")
        self.get_logger().info("")
        
        return threshold_reached

    def summary(self, error=None):
        """Generate JSON summary of positioning results"""
        status = 'success' if error is None else 'error'
        return {
            'initial_height': self.initial_height,
            'adjusted_height': self.adjusted_height,
            'final_height': self.final_height,
            'final_force': self.final_force,
            'force_threshold': self.args.force_threshold,
            'status': status,
            'error': error
        }

def parse_args():
    p = argparse.ArgumentParser(description='Lift Robot Server Positioning')
    p.add_argument('--pushrod-offset-mm', type=float, default=15.0, 
                   help='Pushrod offset in mm above initial height (default: 15.0)')
    p.add_argument('--pushrod-adjust-timeout', type=float, default=10.0, 
                   help='Timeout for pushrod height adjustment in seconds (default: 10.0)')
    p.add_argument('--height-tolerance', type=float, default=0.5, 
                   help='Height tolerance in mm for considering target reached (default: 0.5)')
    p.add_argument('--force-threshold', type=float, default=750.0, 
                   help='Force threshold in Newtons for server contact detection (default: 750.0)')
    p.add_argument('--dry-run', action='store_true', 
                   help='Simulate without sending motion commands')
    return p.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = PositioningNode(args)
    
    # Wait for publishers to establish connections
    node.get_logger().info("Waiting for topic connections...")
    time.sleep(1.0)
    
    # Verify connections
    platform_pub_count = node.platform_cmd_pub.get_subscription_count()
    pushrod_pub_count = node.pushrod_cmd_pub.get_subscription_count()
    node.get_logger().info(f"Platform command subscribers: {platform_pub_count}")
    node.get_logger().info(f"Pushrod command subscribers: {pushrod_pub_count}")
    
    if platform_pub_count == 0:
        node.get_logger().warn("No subscribers for platform command topic!")
    if pushrod_pub_count == 0:
        node.get_logger().warn("No subscribers for pushrod command topic!")
    
    try:
        node.perform_initialization()
        success = node.raise_to_server()
        
        result = node.summary()
        print(json.dumps(result, indent=2))
        
    except Exception as e:
        result = node.summary(error=str(e))
        print(json.dumps(result, indent=2))
        node.get_logger().error(f"Positioning failed: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
