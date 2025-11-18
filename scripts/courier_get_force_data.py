#!/usr/bin/env python3
"""
Script to get force sensor data from lift robot web interface.
Reads Right (CH2) and Left (CH3) force sensor values.
"""

import requests
import json
import sys

def get_force_data(host="192.168.1.3", port=8090, timeout=10):
    """
    Get force sensor data from lift robot web interface.
    
    Args:
        host (str): Host IP address
        port (int): Port number
        timeout (int): Request timeout in seconds
    
    Returns:
        float: right_force value or None if failed
    """
    try:
        url = f"http://{host}:{port}/api/latest"
        response = requests.get(url, timeout=timeout)
        
        if response.status_code == 200:
            data = response.json()
            
            # Extract right force sensor value
            right_force = data.get('right_force_sensor', None)
            
            return right_force
            
        else:
            return None
            
    except Exception as e:
        return None

def main():
    """Main function for command line usage."""
    # Fixed default values
    host = '192.168.1.3'
    port = 8090
    timeout = 10
    
    right_force = get_force_data(host, port, timeout)
    
    if right_force is not None:
        print(right_force)
    else:
        print("Error: Failed to get force data")
        sys.exit(1)

if __name__ == '__main__':
    main()