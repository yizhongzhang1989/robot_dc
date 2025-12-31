#!/usr/bin/env python3
"""
Find Femto Bolt camera video device and display information.
"""

import subprocess
import re
import sys


def run_command(cmd):
    """Run shell command and return output."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.stdout
    except Exception as e:
        return f"Error: {e}"


def get_video_devices():
    """Get list of all video devices."""
    output = run_command("ls /dev/video* 2>/dev/null")
    if output:
        devices = [line.strip() for line in output.split('\n') if line.strip()]
        return devices
    return []


def get_device_info(device):
    """Get detailed information about a video device."""
    cmd = f"v4l2-ctl --device={device} --info 2>/dev/null"
    output = run_command(cmd)
    return output


def get_device_formats(device):
    """Get supported formats for a video device."""
    cmd = f"v4l2-ctl --device={device} --list-formats 2>/dev/null"
    output = run_command(cmd)
    return output


def get_device_formats_ext(device):
    """Get supported formats with resolutions for a video device."""
    cmd = f"v4l2-ctl --device={device} --list-formats-ext 2>/dev/null"
    output = run_command(cmd)
    return output


def is_femto_bolt(device_info):
    """Check if device is Femto Bolt camera."""
    return 'Orbbec Femto Bolt' in device_info or 'Femto Bolt' in device_info


def parse_formats(formats_output):
    """Parse format information."""
    formats = []
    for line in formats_output.split('\n'):
        if "'YUYV'" in line or "'MJPG'" in line or "'RGB'" in line or "'BGR'" in line:
            formats.append(line.strip())
    return formats


def parse_resolutions(formats_ext_output):
    """Parse resolution information."""
    resolutions = []
    lines = formats_ext_output.split('\n')
    for line in lines:
        if 'Size: Discrete' in line:
            match = re.search(r'(\d+)x(\d+)', line)
            if match:
                resolutions.append(f"{match.group(1)}x{match.group(2)}")
    return resolutions


def main():
    print("=" * 70)
    print("Searching for Femto Bolt Camera")
    print("=" * 70)
    print()
    
    devices = get_video_devices()
    
    if not devices:
        print("❌ No video devices found!")
        sys.exit(1)
    
    print(f"Found {len(devices)} video device(s)")
    print()
    
    femto_devices = []
    
    for device in devices:
        info = get_device_info(device)
        
        if is_femto_bolt(info):
            femto_devices.append(device)
            
            print(f"✅ Found Femto Bolt: {device}")
            print("-" * 70)
            
            # Extract card type
            for line in info.split('\n'):
                if 'Card type' in line:
                    print(f"   {line.strip()}")
                elif 'Driver name' in line:
                    print(f"   {line.strip()}")
            
            # Get formats
            formats_output = get_device_formats(device)
            formats = parse_formats(formats_output)
            
            if formats:
                print(f"\n   Supported formats:")
                for fmt in formats:
                    print(f"      {fmt}")
            
            # Get resolutions
            formats_ext = get_device_formats_ext(device)
            resolutions = parse_resolutions(formats_ext)
            
            if resolutions:
                print(f"\n   Supported resolutions:")
                for res in resolutions[:5]:  # Show first 5
                    print(f"      {res}")
                if len(resolutions) > 5:
                    print(f"      ... and {len(resolutions) - 5} more")
            
            print()
    
    if not femto_devices:
        print("❌ No Femto Bolt camera found!")
        print()
        print("Available devices:")
        for device in devices:
            info = get_device_info(device)
            for line in info.split('\n'):
                if 'Card type' in line:
                    card = line.split(':', 1)[1].strip() if ':' in line else 'Unknown'
                    print(f"   {device}: {card}")
        sys.exit(1)
    
    # Find the RGB/color device (usually has YUYV or MJPG format)
    color_device = None
    for device in femto_devices:
        formats_output = get_device_formats(device)
        if 'YUYV' in formats_output or 'MJPG' in formats_output:
            color_device = device
            break
    
    if not color_device and femto_devices:
        color_device = femto_devices[0]
    
    # Print usage instructions
    print("=" * 70)
    print("How to use with ROS 2 usb_cam")
    print("=" * 70)
    print()
    
    if color_device:
        device_num = color_device.replace('/dev/video', '')
        
        print("1. Install usb_cam (if not already installed):")
        print("   sudo apt install ros-humble-usb-cam")
        print()
        
        print("2. Run usb_cam node:")
        print(f"""
   ros2 run usb_cam usb_cam_node_exe --ros-args \\
     -p video_device:={color_device} \\
     -p image_width:=1280 \\
     -p image_height:=720 \\
     -p framerate:=30.0 \\
     -p pixel_format:=yuyv \\
     -r image_raw:=image_raw
""")
        
        print("3. To publish to a custom topic (e.g., /ur15_wrist_camera/image_raw):")
        print(f"""
   ros2 run usb_cam usb_cam_node_exe --ros-args \\
     -p video_device:={color_device} \\
     -p image_width:=1280 \\
     -p image_height:=720 \\
     -p framerate:=30.0 \\
     -p pixel_format:=yuyv \\
     -r __ns:=/ur15_wrist_camera \\
     -r image_raw:=image_raw
""")
        
        print("4. View the image stream:")
        print("   ros2 run rqt_image_view rqt_image_view")
        print()
        
        print("5. Check topic rate:")
        print("   ros2 topic hz /image_raw")
        print("   # or")
        print("   ros2 topic hz /ur15_wrist_camera/image_raw")
        print()
    
    print("=" * 70)
    print(f"Summary: Found {len(femto_devices)} Femto Bolt device(s)")
    if color_device:
        print(f"Color/RGB device: {color_device}")
    print("=" * 70)


if __name__ == '__main__':
    main()
