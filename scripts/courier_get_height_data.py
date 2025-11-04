#!/usr/bin/env python3
"""
Script to get height data from lift robot web interface.
Reads current platform height in millimeters.
"""

import requests
import json
import sys
import time

def get_height_data(host="192.168.1.3", port=8090, timeout=10):
    """
    Get height data from lift robot web interface.
    
    Args:
        host (str): Host IP address
        port (int): Port number
        timeout (int): Request timeout in seconds
    
    Returns:
        float: Platform current height in millimeters, or None if failed
    """
    try:
        url = f"http://{host}:{port}/api/latest"
        response = requests.get(url, timeout=timeout)
        
        if response.status_code == 200:
            data = response.json()
            
            # Return platform_current_height if available
            if 'platform_status' in data:
                platform_status = data['platform_status']
                return platform_status.get('current_height', None)
            
            # Fallback to height field
            return data.get('height', None)
            
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
    
    height = get_height_data(host, port, timeout)
    
    if height is not None:
        print(height)
    else:
        print("Error: Failed to get height data")
        sys.exit(1)

if __name__ == '__main__':
    main()