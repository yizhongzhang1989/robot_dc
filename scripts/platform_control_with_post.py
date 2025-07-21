#!/usr/bin/env python3
"""
Platform Control Script using HTTP POST requests
Usage examples:
    python3 platform_control_with_post.py up 3.0
    python3 platform_control_with_post.py down 2.5
    python3 platform_control_with_post.py forward 1.5
    python3 platform_control_with_post.py backward 4.0
    python3 platform_control_with_post.py stop_all
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


def send_timed_movement(direction, duration):
    """Send timed movement command"""
    url = 'http://localhost:8000/api/platform/timed_move'
    data = {
        "direction": direction,
        "duration": float(duration)
    }
    
    try:
        response = requests.post(url, json=data, timeout=5)
        response.raise_for_status()
        result = response.json()
        print(f"✅ Timed movement started: {direction} for {duration} seconds")
        print(f"📡 Response: {result}")
        return result
    except requests.exceptions.RequestException as e:
        print(f"❌ Error sending timed movement: {e}")
        return None


def stop_all_movements():
    """Stop all active timed movements"""
    return send_platform_command("stop_all")


def main():
    parser = argparse.ArgumentParser(
        description="Control platform movement via HTTP POST",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s up 3.0        # Move up for 3.0 seconds
  %(prog)s down 2.5      # Move down for 2.5 seconds  
  %(prog)s forward 1.5   # Move forward for 1.5 seconds
  %(prog)s backward 4.0  # Move backward for 4.0 seconds
  %(prog)s stop_all      # Stop all movements
        """
    )
    
    parser.add_argument('direction', 
                       choices=['up', 'down', 'forward', 'backward', 'stop_all'],
                       help='Movement direction or stop command')
    parser.add_argument('duration', 
                       type=float, 
                       nargs='?',
                       help='Duration in seconds (not needed for stop_all)')
    
    args = parser.parse_args()
    
    print(f"🤖 Platform Control Script")
    print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 50)
    
    if args.direction == 'stop_all':
        print("🛑 Stopping all active movements...")
        result = stop_all_movements()
    else:
        if args.duration is None:
            print(f"❌ Error: Duration is required for {args.direction} movement")
            sys.exit(1)
        
        if args.duration <= 0:
            print(f"❌ Error: Duration must be positive, got {args.duration}")
            sys.exit(1)
        
        print(f"🚀 Starting {args.direction} movement for {args.duration} seconds...")
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
        ("up", 2.0),
        ("down", 2.0), 
        ("forward", 1.5),
        ("backward", 1.5)
    ]
    
    for direction, duration in movements:
        print(f"🚀 Moving {direction} for {duration} seconds...")
        result = send_timed_movement(direction, duration)
        if result:
            # Wait for movement to complete plus a small buffer
            time.sleep(duration + 0.5)
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
