#!/usr/bin/env python3
"""
Platform Control Script using HTTP POST requests
Usage examples:
    python3 platform_control_with_post.py up 3000        # Move up for 3000ms (3 seconds)
    python3 platform_control_with_post.py down 2500      # Move down for 2500ms (2.5 seconds)
    python3 platform_control_with_post.py forward 1500   # Move forward for 1500ms (1.5 seconds)
    python3 platform_control_with_post.py backward 4000  # Move backward for 4000ms (4 seconds)
    python3 platform_control_with_post.py up             # Move up continuously (manual mode)
    python3 platform_control_with_post.py stop           # Stop any ongoing movement
    python3 platform_control_with_post.py stop_all       # Stop all movements
"""

import requests
import sys
import time
import argparse


def send_platform_command(command, value=None):
    """Send basic platform command"""
    url = 'http://localhost:8000/api/platform/cmd'
    data = {"command": command}
    if value is not None:
        data["value"] = value
    
    try:
        response = requests.post(url, json=data, timeout=5)
        response.raise_for_status()
        result = response.json()
        print(f"✅ Command sent: {command}" + (f" {value}" if value else ""))
        print(f"📡 Response: {result}")
        return result
    except requests.exceptions.RequestException as e:
        print(f"❌ Error sending command: {e}")
        return None


def send_continuous_movement(direction):
    """Send continuous movement command (manual mode)"""
    # For continuous movement, send the basic platform command with value 1
    return send_platform_command(direction, 1)


def send_timed_movement(direction, duration_ms):
    """Send timed movement command"""
    url = 'http://localhost:8000/api/platform/timed_move'
    # Convert milliseconds to seconds for the backend API
    duration_seconds = duration_ms / 1000.0
    data = {
        "direction": direction,
        "duration": duration_seconds
    }
    
    try:
        response = requests.post(url, json=data, timeout=5)
        response.raise_for_status()
        result = response.json()
        print(f"✅ Timed movement started: {direction} for {duration_ms}ms ({duration_seconds}s)")
        print(f"📡 Response: {result}")
        return result
    except requests.exceptions.RequestException as e:
        print(f"❌ Error sending timed movement: {e}")
        return None


def stop_movement():
    """Stop any ongoing movement (both continuous and timed)"""
    # For stopping continuous movements, we need to send stop commands for all directions
    # Each direction command with value 0 will stop that direction
    directions = ['up', 'down', 'forward', 'backward']
    results = []
    
    print("🛑 Stopping continuous movements in all directions...")
    for direction in directions:
        result = send_platform_command(direction, 0)
        if result:
            results.append(f"{direction}: stopped")
        else:
            results.append(f"{direction}: failed")
    
    # Also stop any timed movements
    print("🛑 Stopping timed movements...")
    timed_result = send_platform_command("stop_all")
    
    if timed_result:
        results.append("timed movements: stopped")
    else:
        results.append("timed movements: failed")
    
    print(f"📋 Stop results: {', '.join(results)}")
    return {"results": results, "success": True}


def stop_all_movements():
    """Stop all active movements (both continuous and timed)"""
    # This function now calls the enhanced stop_movement function
    # which handles both continuous and timed movements
    print("🛑 Stopping ALL movements (continuous + timed)...")
    return stop_movement()


def main():
    parser = argparse.ArgumentParser(
        description="Control platform movement via HTTP POST",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s up 3000       # Move up for 3000ms (3 seconds)
  %(prog)s down 2500     # Move down for 2500ms (2.5 seconds)  
  %(prog)s forward 1500  # Move forward for 1500ms (1.5 seconds)
  %(prog)s backward 4000 # Move backward for 4000ms (4 seconds)
  %(prog)s up            # Move up continuously (manual mode)
  %(prog)s stop          # Stop any ongoing movement
  %(prog)s stop_all      # Stop all movements
        """
    )
    
    parser.add_argument('direction', 
                       choices=['up', 'down', 'forward', 'backward', 'stop', 'stop_all'],
                       help='Movement direction or stop command')
    parser.add_argument('duration', 
                       type=int, 
                       nargs='?',
                       help='Duration in milliseconds (not needed for stop/stop_all or continuous movement)')
    
    args = parser.parse_args()
    
    print(f"🤖 Platform Control Script")
    print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 50)
    
    if args.direction == 'stop_all':
        print("🛑 Stopping all active movements...")
        result = stop_all_movements()
    elif args.direction == 'stop':
        print("🛑 Stopping current movement...")
        result = stop_movement()
    else:
        # Handle movement commands
        if args.duration is None:
            # Continuous movement (manual mode)
            print(f"🚀 Starting continuous {args.direction} movement (manual mode)...")
            print("💡 Use 'stop' command to stop the movement")
            result = send_continuous_movement(args.direction)
        else:
            # Timed movement
            if args.duration <= 0:
                print(f"❌ Error: Duration must be positive, got {args.duration}ms")
                sys.exit(1)
            
            print(f"🚀 Starting {args.direction} movement for {args.duration}ms...")
            result = send_timed_movement(args.direction, args.duration)
    
    if result is None:
        print("❌ Command failed!")
        sys.exit(1)
    else:
        print("✅ Command completed successfully!")


def demo_sequence():
    """Demo function showing various platform movements"""
    print("🎮 Starting Platform Control Demo...")
    print("-" * 50)
    
    movements = [
        ("up", 2000),      # 2 seconds
        ("down", 2000),    # 2 seconds
        ("forward", 1500), # 1.5 seconds
        ("backward", 1500) # 1.5 seconds
    ]
    
    for direction, duration_ms in movements:
        print(f"🚀 Moving {direction} for {duration_ms}ms...")
        result = send_timed_movement(direction, duration_ms)
        if result:
            # Wait for movement to complete plus a small buffer
            time.sleep((duration_ms / 1000.0) + 0.5)
        else:
            print(f"❌ Failed to start {direction} movement")
            break
    
    print("🛑 Stopping all movements...")
    stop_all_movements()
    print("✅ Demo completed!")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("No arguments provided. Run with --help for usage information.")
        print("Or uncomment the demo_sequence() call below to run a demo.")
        # Uncomment the next line to run demo:
        # demo_sequence()
    else:
        main()
