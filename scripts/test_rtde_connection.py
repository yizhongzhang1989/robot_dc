#!/usr/bin/env python3
"""
RTDE Connection Test Script
Test if the RTDE connection to the robot is working properly
"""

from rtde_receive import RTDEReceiveInterface
import time
import socket
import sys

# Robot IP address
ROBOT_IP = "192.168.1.15"
RTDE_PORT = 30004

def check_network_connection(ip, port, timeout=2):
    """Check if the robot is reachable on the network"""
    print(f"[1/3] Testing network connectivity to {ip}:{port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        result = sock.connect_ex((ip, port))
        sock.close()
        
        if result == 0:
            print(f"    [OK] Port {port} is reachable")
            return True
        else:
            print(f"    [FAIL] Cannot connect to port {port}")
            print(f"    Error code: {result}")
            return False
    except socket.gaierror:
        print(f"    [FAIL] Hostname resolution failed")
        print(f"    Check if IP address '{ip}' is valid")
        return False
    except socket.timeout:
        print(f"    [FAIL] Connection timeout")
        print(f"    Robot may be unreachable or network is slow")
        return False
    except Exception as e:
        print(f"    [FAIL] Network error: {e}")
        return False

def test_rtde_connection(ip):
    """Test RTDE connection and protocol"""
    print(f"\n[2/3] Establishing RTDE connection...")
    try:
        rtde_r = RTDEReceiveInterface(ip, verbose=True)
        print("    [OK] RTDE connection established")
        return rtde_r
    except RuntimeError as e:
        print(f"    [FAIL] RTDE runtime error: {e}")
        if "protocol" in str(e).lower():
            print("    This might be a protocol version mismatch")
        elif "timeout" in str(e).lower():
            print("    RTDE service may not be responding")
        return None
    except Exception as e:
        print(f"    [FAIL] Unexpected error: {type(e).__name__}: {e}")
        return None

def test_data_reading(rtde_r):
    """Test reading data from RTDE"""
    print(f"\n[3/3] Testing data reading...")
    try:
        actual_q = rtde_r.getActualQ()
        if actual_q is not None and len(actual_q) > 0:
            print(f"    [OK] Successfully read joint angles: {actual_q}")
            return True
        else:
            print(f"    [FAIL] Received invalid data: {actual_q}")
            return False
    except Exception as e:
        print(f"    [FAIL] Cannot read data: {type(e).__name__}: {e}")
        return False

def continuous_monitoring(rtde_r):
    """Continuously monitor joint angles"""
    print("\n" + "="*60)
    print("All checks passed! Starting continuous monitoring...")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    count = 0
    try:
        while True:
            actual_q = rtde_r.getActualQ()
            count += 1
            print(f"[{count}] Joint angles: {actual_q}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print(f"\n\nMonitoring stopped after {count} readings")

def main():
    print("="*60)
    print("RTDE Connection Test")
    print("="*60)
    
    # Step 1: Check network connectivity
    if not check_network_connection(ROBOT_IP, RTDE_PORT):
        print("\n[RESULT] Network connectivity test FAILED")
        print("\nTroubleshooting steps:")
        print("  1. Verify robot IP address is correct")
        print("  2. Check if robot is powered on")
        print("  3. Ensure robot and PC are on the same network")
        print("  4. Try ping: ping", ROBOT_IP)
        sys.exit(1)
    
    # Step 2: Test RTDE connection
    rtde_r = test_rtde_connection(ROBOT_IP)
    if rtde_r is None:
        print("\n[RESULT] RTDE connection test FAILED")
        print("\nTroubleshooting steps:")
        print("  1. Check if RTDE is enabled on the robot")
        print("  2. Verify no other program is using RTDE")
        print("  3. Try restarting the robot controller")
        print("  4. Check robot teach pendant for any errors")
        sys.exit(1)
    
    # Step 3: Test data reading
    if not test_data_reading(rtde_r):
        print("\n[RESULT] Data reading test FAILED")
        print("\nTroubleshooting steps:")
        print("  1. Robot may be in an error state")
        print("  2. Check robot teach pendant for warnings")
        print("  3. Try restarting RTDE service on robot")
        sys.exit(1)
    
    # All tests passed, start continuous monitoring
    continuous_monitoring(rtde_r)

if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
