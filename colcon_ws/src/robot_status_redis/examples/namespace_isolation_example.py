#!/usr/bin/env python3
"""
Demonstration of Redis namespace isolation for robot_status_redis.

This example shows how different applications can use the same Redis server
without interfering with each other's data.

Key Concepts:
1. Data Persistence: Redis data persists after process termination
2. Namespace Isolation: Use key_prefix or db parameters to isolate data
3. Multiple Applications: Different processes can share Redis safely

Run this example multiple times to see data persistence.
"""

from robot_status_redis.client_utils import RobotStatusClient
import time

def demonstrate_isolation():
    """Show how different applications can isolate their data."""
    
    print("\n" + "="*70)
    print("Redis Namespace Isolation Demonstration")
    print("="*70)
    
    # Application 1: Uses default prefix 'robot_status'
    print("\n1. Application 1 (default namespace: 'robot_status')")
    app1_client = RobotStatusClient(key_prefix='robot_status')
    app1_client.set_status('robot1', 'position', {'x': 1.0, 'y': 2.0})
    app1_client.set_status('robot1', 'battery', 85)
    print("   - Set robot1 position and battery")
    
    # Application 2: Uses custom prefix 'my_app'
    print("\n2. Application 2 (custom namespace: 'my_app')")
    app2_client = RobotStatusClient(key_prefix='my_app')
    app2_client.set_status('robot1', 'position', {'x': 10.0, 'y': 20.0})
    app2_client.set_status('robot1', 'temperature', 25.5)
    print("   - Set robot1 position and temperature")
    
    # Application 3: Uses different database (complete isolation)
    print("\n3. Application 3 (different database: db=1)")
    app3_client = RobotStatusClient(db=1, key_prefix='robot_status')
    app3_client.set_status('robot1', 'position', {'x': 100.0, 'y': 200.0})
    print("   - Set robot1 position in separate database")
    
    print("\n" + "-"*70)
    print("Reading data back - each application sees only its own data:")
    print("-"*70)
    
    # Read from each application
    print("\n1. Application 1 reads:")
    pos1 = app1_client.get_status('robot1', 'position')
    bat1 = app1_client.get_status('robot1', 'battery')
    temp1 = app1_client.get_status('robot1', 'temperature')  # Won't exist
    print(f"   - Position: {pos1}")
    print(f"   - Battery: {bat1}%")
    print(f"   - Temperature: {temp1} (not set by app1)")
    
    print("\n2. Application 2 reads:")
    pos2 = app2_client.get_status('robot1', 'position')
    bat2 = app2_client.get_status('robot1', 'battery')  # Won't exist
    temp2 = app2_client.get_status('robot1', 'temperature')
    print(f"   - Position: {pos2}")
    print(f"   - Battery: {bat2} (not set by app2)")
    print(f"   - Temperature: {temp2}°C")
    
    print("\n3. Application 3 reads:")
    pos3 = app3_client.get_status('robot1', 'position')
    bat3 = app3_client.get_status('robot1', 'battery')  # Won't exist
    print(f"   - Position: {pos3}")
    print(f"   - Battery: {bat3} (different database)")
    
    print("\n" + "="*70)
    print("Data Persistence Demonstration")
    print("="*70)
    print("\nTo verify data persists after process termination:")
    print("1. Run this script: python3 namespace_isolation_example.py")
    print("2. Exit the script (Ctrl+C or let it finish)")
    print("3. Run it again - you'll see the same data!")
    print("\nTo clear data, use Redis CLI:")
    print("   redis-cli FLUSHDB     # Clear current database")
    print("   redis-cli FLUSHALL    # Clear all databases")
    
    print("\n" + "="*70)
    print("Best Practices")
    print("="*70)
    print("\n1. Use key_prefix for light isolation (same database):")
    print("   client = RobotStatusClient(key_prefix='my_app')")
    print("\n2. Use db parameter for complete isolation (separate database):")
    print("   client = RobotStatusClient(db=1)  # db 0-15")
    print("\n3. Clean up when done:")
    print("   client.delete_status('robot1')  # Delete namespace")
    print("\n4. Monitor Redis usage:")
    print("   redis-cli INFO memory  # Check memory usage")
    print("   redis-cli DBSIZE       # Count keys")
    print("="*70)


def demonstrate_persistence():
    """Show that data persists across multiple runs."""
    print("\n" + "="*70)
    print("Checking for existing data from previous runs...")
    print("="*70)
    
    client = RobotStatusClient(key_prefix='persistence_demo')
    
    # Try to read counter
    counter = client.get_status('demo', 'run_counter')
    
    if counter is None:
        print("\nNo previous data found. This is the first run.")
        counter = 1
    else:
        counter += 1
        print(f"\nFound data from previous run! This is run #{counter}")
    
    # Update counter
    client.set_status('demo', 'run_counter', counter)
    client.set_status('demo', 'last_run_time', time.strftime('%Y-%m-%d %H:%M:%S'))
    
    print(f"Updated run counter to: {counter}")
    print("Run this script again to see the counter increment!")
    
    # Show all data
    all_data = client.list_status('demo')
    if 'demo' in all_data:
        print(f"\nAll persistent data:")
        for key, value_dict in all_data['demo'].items():
            if isinstance(value_dict, dict) and 'json' in value_dict:
                print(f"  - {key}: {value_dict['json']}")
            else:
                print(f"  - {key}: {value_dict}")


if __name__ == '__main__':
    try:
        demonstrate_isolation()
        print("\n")
        demonstrate_persistence()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
