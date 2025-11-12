#!/usr/bin/env python3
"""
Lift Robot Server Positioning Script (HTTP Version)

Task:
    Phase 1 - Initialization:
        1. Platform DOWN for 15 seconds
        2. Pushrod DOWN for 7 seconds
        3. Read current height after both stopped
        4. Use pushrod goto_height to adjust to (current_height + 18mm)
    
    Phase 2 - Raise to Server:
        1. Platform force_up with target force (default 140N)
        2. Monitor via HTTP status API until task completes
    
    Phase 3 - Apply Final Force:
        1. Wait 10 seconds (default delay)
        2. Platform force_up with higher target force (default 460N)
        3. Monitor via HTTP status API until task completes

Uses HTTP API instead of ROS2 topics for better decoupling.

Produces JSON summary on stdout:
{
    "initial_height": <float>,
    "adjusted_height": <float>,
    "final_height": <float>,
    "final_force": <float>,
    "force_threshold": <float>,
    "force_threshold_final": <float>,
    "status": "success"|"error",
    "error": <str or null>
}

Run directly (no ROS2 workspace needed):
    python3 scripts/lift_robot_server_positioning.py
"""
import requests
import json
import time
import sys
import argparse

# Web server base URL
LIFT_WEB_BASE = "http://localhost:8090"


def send_command(target, command, **kwargs):
    """Send command via HTTP POST"""
    try:
        url = f"{LIFT_WEB_BASE}/api/cmd"
        payload = {
            'command': command,
            'target': target,
            **kwargs
        }
        
        response = requests.post(url, json=payload, timeout=5)
        
        if response.status_code == 200:
            return {"success": True, "response": response.json()}
        else:
            return {"success": False, "error": f"HTTP {response.status_code}: {response.text}"}
            
    except Exception as e:
        return {"success": False, "error": str(e)}


def get_status():
    """Query status via HTTP GET"""
    try:
        url = f"{LIFT_WEB_BASE}/api/status"
        response = requests.get(url, timeout=2)
        
        if response.status_code == 200:
            return response.json()
        else:
            return None
            
    except Exception as e:
        print(f"Status query error: {e}", file=sys.stderr)
        return None


def get_latest_sensor_data():
    """Get latest sensor data (height and force)"""
    try:
        url = f"{LIFT_WEB_BASE}/api/latest"
        response = requests.get(url, timeout=2)
        
        if response.status_code == 200:
            return response.json()
        else:
            return None
            
    except Exception as e:
        return None


def wait_task_complete(target, timeout=90, poll_interval=0.2):
    """Wait for platform/pushrod task to complete"""
    start_time = time.time()
    last_state = None
    
    print(f"Waiting for {target} task completion...", file=sys.stderr)
    
    while time.time() - start_time < timeout:
        status = get_status()
        
        if status and target in status:
            task_info = status[target]
            task_state = task_info.get('task_state', 'unknown')
            
            if task_state != last_state:
                elapsed = time.time() - start_time
                print(f"  [{elapsed:.1f}s] {target}: {task_state}", file=sys.stderr)
                last_state = task_state
            
            if task_state == 'completed':
                return {
                    "success": True,
                    "task_info": task_info
                }
            
            elif task_state == 'emergency_stop':
                return {
                    "success": False,
                    "error": "Emergency stop triggered"
                }
            
            elif task_state == 'idle' and time.time() - start_time > 1.0:
                # Returned to idle, task finished
                return {
                    "success": True,
                    "task_info": task_info
                }
        
        time.sleep(poll_interval)
    
    return {
        "success": False,
        "error": f"Timeout after {timeout}s"
    }


def perform_initialization(args):
    """Phase 1: Initialization"""
    print("Phase 1: Initialization", file=sys.stderr)
    
    # Platform down for 15s
    print("  Platform down 15s...", file=sys.stderr)
    result = send_command('platform', 'down')
    if not result['success']:
        raise RuntimeError(f"Platform down failed: {result.get('error')}")
    
    time.sleep(15.0)
    send_command('platform', 'stop')
    
    # Pushrod down for 7s
    print("  Pushrod down 7s...", file=sys.stderr)
    result = send_command('pushrod', 'down')
    if not result['success']:
        raise RuntimeError(f"Pushrod down failed: {result.get('error')}")
    
    time.sleep(7.0)
    send_command('pushrod', 'stop')
    
    # Wait for settle
    print("  Settling...", file=sys.stderr)
    time.sleep(2.0)
    
    # Get current height
    sensor_data = None
    for _ in range(10):
        sensor_data = get_latest_sensor_data()
        if sensor_data and 'height' in sensor_data:
            break
        time.sleep(0.5)
    
    if not sensor_data or 'height' not in sensor_data:
        raise RuntimeError("No height data received")
    
    initial_height = sensor_data['height']
    adjusted_height = initial_height + args.pushrod_offset_mm
    
    print(f"  Initial height: {initial_height:.2f}mm", file=sys.stderr)
    print(f"  Adjusting pushrod to {adjusted_height:.2f}mm (+{args.pushrod_offset_mm}mm)", file=sys.stderr)
    
    # Pushrod goto_height
    result = send_command('pushrod', 'goto_height', target_height=adjusted_height)
    if not result['success']:
        raise RuntimeError(f"Pushrod goto_height failed: {result.get('error')}")
    
    # Wait for pushrod to complete
    wait_result = wait_task_complete('pushrod', timeout=args.pushrod_adjust_timeout)
    if not wait_result['success']:
        raise RuntimeError(f"Pushrod adjustment failed: {wait_result.get('error')}")
    
    print(f"  ✅ Initialization complete", file=sys.stderr)
    return initial_height, adjusted_height


def raise_to_server(args):
    """Phase 2: Raise to Server (force_up to threshold)"""
    print(f"Phase 2: Raise to server (target force: {args.force_threshold}N)", file=sys.stderr)
    
    result = send_command('platform', 'force_up', target_force=args.force_threshold)
    if not result['success']:
        raise RuntimeError(f"Platform force_up failed: {result.get('error')}")
    
    # Wait for completion
    wait_result = wait_task_complete('platform', timeout=args.force_timeout)
    if not wait_result['success']:
        raise RuntimeError(f"Platform force_up failed: {wait_result.get('error')}")
    
    # Get final sensor data
    sensor_data = get_latest_sensor_data()
    final_height = sensor_data.get('height') if sensor_data else None
    combined_force = sensor_data.get('combined_force_sensor') if sensor_data else None
    
    print(f"  ✅ Phase 2 complete: force={combined_force}N, height={final_height}mm", file=sys.stderr)
    return final_height, combined_force


def apply_final_force(args):
    """Phase 3: Apply Final Force"""
    print(f"Phase 3: Delay {args.force_delay}s, then force_up to {args.force_threshold_final}N", file=sys.stderr)
    
    time.sleep(args.force_delay)
    
    result = send_command('platform', 'force_up', target_force=args.force_threshold_final)
    if not result['success']:
        raise RuntimeError(f"Platform force_up (final) failed: {result.get('error')}")
    
    # Wait for completion
    wait_result = wait_task_complete('platform', timeout=args.force_timeout)
    if not wait_result['success']:
        raise RuntimeError(f"Platform force_up (final) failed: {wait_result.get('error')}")
    
    # Get final sensor data
    sensor_data = get_latest_sensor_data()
    final_height = sensor_data.get('height') if sensor_data else None
    combined_force = sensor_data.get('combined_force_sensor') if sensor_data else None
    force1 = sensor_data.get('right_force_sensor') if sensor_data else None
    force2 = sensor_data.get('left_force_sensor') if sensor_data else None
    
    print(f"  ✅ Phase 3 complete: force={combined_force}N, height={final_height}mm", file=sys.stderr)
    return final_height, combined_force, force1, force2


def parse_args():
    p = argparse.ArgumentParser(description='Lift Robot Server Positioning (HTTP)')
    p.add_argument('--pushrod-offset-mm', type=float, default=18.0,
                   help='Pushrod offset in mm above initial height (default: 18.0)')
    p.add_argument('--pushrod-adjust-timeout', type=float, default=10.0,
                   help='Timeout for pushrod height adjustment in seconds (default: 10.0)')
    p.add_argument('--force-threshold', type=float, default=140.0,
                   help='Force threshold in Newtons for server contact detection (default: 140.0)')
    p.add_argument('--force-threshold-final', type=float, default=460.0,
                   help='Final force threshold in Newtons for firm contact (default: 460.0)')
    p.add_argument('--force-delay', type=float, default=10.0,
                   help='Delay in seconds before applying final force (default: 10.0)')
    p.add_argument('--force-timeout', type=float, default=90.0,
                   help='Timeout (s) for force_up phase before abort (default: 90.0)')
    p.add_argument('--web-base', type=str, default='http://localhost:8090',
                   help='Lift web server base URL (default: http://localhost:8090)')
    return p.parse_args()


def main():
    global LIFT_WEB_BASE
    
    args = parse_args()
    LIFT_WEB_BASE = args.web_base
    
    initial_height = None
    adjusted_height = None
    final_height = None
    final_force = None
    force1 = None
    force2 = None
    error = None
    
    try:
        # Phase 1
        initial_height, adjusted_height = perform_initialization(args)
        
        # Phase 2
        height2, force2_val = raise_to_server(args)
        
        # Phase 3
        final_height, final_force, force1, force2 = apply_final_force(args)
        
        print("✅ All phases completed successfully", file=sys.stderr)
        
    except Exception as e:
        error = str(e)
        print(f"❌ Error: {error}", file=sys.stderr)
    
    # Output JSON summary to stdout
    result = {
        'initial_height': initial_height,
        'adjusted_height': adjusted_height,
        'final_height': final_height,
        'final_force': final_force,
        'force1': force1,
        'force2': force2,
        'force_threshold': args.force_threshold,
        'force_threshold_final': args.force_threshold_final,
        'status': 'success' if error is None else 'error',
        'error': error
    }
    
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

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
        
        # Force sensor subscriptions (raw values, no filtering)
        # 主力 /force_sensor + 副力 /force_sensor_2 合力用于判定停止
        self.force1 = None
        self.force2 = None
        self.combined_force = None  # force1 + force2 when both available
        self.force1_sub = self.create_subscription(Float32, '/force_sensor', self.force1_cb, 10)
        self.force2_sub = self.create_subscription(Float32, '/force_sensor_2', self.force2_cb, 10)
        
        # Results
        self.initial_height = None      # Height after platform+pushrod down
        self.adjusted_height = None     # Target height for pushrod adjustment (initial + 15mm)
        self.final_height = None        # Height when force threshold reached
        self.final_force = None         # Combined force when stopped
        # Platform status (to detect force control state)
        self.platform_status = None
        self.platform_status_sub = self.create_subscription(String, '/lift_robot_platform/status', self.platform_status_cb, 10)

    def platform_status_cb(self, msg: String):
        try:
            self.platform_status = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Platform status parse error: {e}")

    def height_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            h = data.get('height')
            if h is not None:
                self.height = float(h)
        except Exception as e:
            self.get_logger().warn(f"Height parse error: {e}")

    def _store_force_timestamp(self):
        if not hasattr(self, 'force_timestamps'):
            self.force_timestamps = []
        self.force_timestamps.append(time.time())
        if len(self.force_timestamps) > 100:
            self.force_timestamps.pop(0)

    def _update_combined(self):
        if self.force1 is not None and self.force2 is not None:
            self.combined_force = self.force1 + self.force2
        elif self.force1 is not None:
            self.combined_force = self.force1
        elif self.force2 is not None:
            self.combined_force = self.force2
        else:
            self.combined_force = None

    def force1_cb(self, msg: Float32):
        try:
            self.force1 = float(msg.data)
            self._store_force_timestamp()
            self._update_combined()
        except Exception as e:
            self.get_logger().warn(f"Force1 parse error: {e}")

    def force2_cb(self, msg: Float32):
        try:
            self.force2 = float(msg.data)
            self._store_force_timestamp()
            self._update_combined()
        except Exception as e:
            self.get_logger().warn(f"Force2 parse error: {e}")
    
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
        """Wait for at least one force sensor value to be available"""
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.combined_force is not None:
                return True
        return False

    def perform_initialization(self):
        """
        Phase 1: Initialization
        Platform down, pushrod down, capture height, raise pushrod by offset (default +18mm)
        """
        self.get_logger().info("init: platform down")
        self.publish_cmd(self.platform_cmd_pub, {"command": "down"}, 'platform')
        time.sleep(15.0)
        self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
        self.get_logger().info("init: pushrod down")
        self.publish_cmd(self.pushrod_cmd_pub, {"command": "down"}, 'pushrod')
        time.sleep(7.0)
        self.publish_cmd(self.pushrod_cmd_pub, {"command": "stop"}, 'pushrod')
        self.get_logger().info("init: settle")
        time.sleep(2.0)  # Wait for mechanical settling
        if not self.wait_for_height(timeout=5.0):
            raise RuntimeError("No height data received after initialization")
        self.initial_height = self.height
        # Pushrod adjustment
        self.adjusted_height = self.initial_height + self.args.pushrod_offset_mm
        self.get_logger().info(f"init: pushrod goto {self.adjusted_height:.2f} mm (+{self.args.pushrod_offset_mm}mm)")
        
        self.publish_cmd(
            self.pushrod_cmd_pub,
            {"command": "goto_height", "target_height": self.adjusted_height},
            'pushrod'
        )
        start = time.time()
        while rclpy.ok() and time.time() - start < self.args.pushrod_adjust_timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.height is not None and abs(self.height - self.adjusted_height) <= self.args.height_tolerance:
                break
        self.get_logger().info(f"init: heights initial={self.initial_height:.2f} adjusted={self.adjusted_height:.2f}")

    def raise_to_server(self):
        """
        Phase 2: Raise to Server Position
        1. Issue force_up command to platform controller with target_force
        2. Controller handles pulsing & stopping; here we monitor status/force for completion
        3. Timeout protection
        """
        thresh = self.args.force_threshold
        self.get_logger().info(f"raise: force_up target={thresh:.1f}N")

        # Wait for initial force reading (optional)
        self.wait_for_force(timeout=3.0)

        start = time.time()
        self.publish_cmd(self.platform_cmd_pub, {"command": "force_up", "target_force": thresh}, 'platform')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            elapsed = time.time() - start
            if self.combined_force is not None and self.combined_force >= thresh:
                break
            if elapsed >= self.args.force_timeout:
                self.get_logger().info("raise: timeout stop")
                self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
                break

        # Final record
        self.final_force = self.combined_force
        self.final_height = self.height
        self.get_logger().info(f"raise: complete force={self.final_force} height={self.final_height} duration={time.time()-start:.2f}s")
        return True if (self.final_force is not None and self.final_force >= thresh) else False

    def apply_final_force(self):
        """
        Phase 3: Apply Final Force
        1. Wait for specified delay (default 10s)
        2. Issue force_up command with higher threshold (default 460N)
        3. Monitor until target reached or timeout
        """
        delay = self.args.force_delay
        thresh = self.args.force_threshold_final
        
        self.get_logger().info(f"final_force: delay {delay:.1f}s before applying target={thresh:.1f}N")
        time.sleep(delay)
        
        self.get_logger().info(f"final_force: force_up target={thresh:.1f}N")
        start = time.time()
        self.publish_cmd(self.platform_cmd_pub, {"command": "force_up", "target_force": thresh}, 'platform')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            elapsed = time.time() - start
            if self.combined_force is not None and self.combined_force >= thresh:
                break
            if elapsed >= self.args.force_timeout:
                self.get_logger().info("final_force: timeout stop")
                self.publish_cmd(self.platform_cmd_pub, {"command": "stop"}, 'platform')
                break
        
        # Update final records
        self.final_force = self.combined_force
        self.final_height = self.height
        self.get_logger().info(f"final_force: complete force={self.final_force} height={self.final_height} duration={time.time()-start:.2f}s")
        return True if (self.final_force is not None and self.final_force >= thresh) else False

    def summary(self, error=None):
        """Generate JSON summary of positioning results"""
        status = 'success' if error is None else 'error'
        return {
            'initial_height': self.initial_height,
            'adjusted_height': self.adjusted_height,
            'final_height': self.final_height,
            'final_force': self.final_force,
            'force1': self.force1,
            'force2': self.force2,
            'force_threshold': self.args.force_threshold,
            'force_threshold_final': self.args.force_threshold_final,
            'status': status,
            'error': error
        }

def parse_args():
    p = argparse.ArgumentParser(description='Lift Robot Server Positioning')
    p.add_argument('--pushrod-offset-mm', type=float, default=18.0,
                   help='Pushrod offset in mm above initial height (default: 18.0)')
    p.add_argument('--pushrod-adjust-timeout', type=float, default=10.0,
                   help='Timeout for pushrod height adjustment in seconds (default: 10.0)')
    p.add_argument('--height-tolerance', type=float, default=0.5,
                   help='Height tolerance in mm for considering target reached (default: 0.5)')
    p.add_argument('--force-threshold', type=float, default=140.0,
                   help='Force threshold in Newtons for server contact detection (default: 140.0)')
    p.add_argument('--force-threshold-final', type=float, default=460.0,
                   help='Final force threshold in Newtons for firm contact (default: 460.0)')
    p.add_argument('--force-delay', type=float, default=10.0,
                   help='Delay in seconds before applying final force (default: 10.0)')
    p.add_argument('--force-timeout', type=float, default=90.0,
                   help='Timeout (s) for force_up phase before abort (default: 90.0)')
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
        if success:
            node.apply_final_force()
        
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
