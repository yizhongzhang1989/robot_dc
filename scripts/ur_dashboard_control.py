#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR15 Robot Dashboard Control Script
Control robot power on/off, brake release, program play/stop/pause operations
"""

import socket
import time
import sys


class URDashboardControl:
    """UR Robot Dashboard Control Class"""
    
    def __init__(self, host="192.168.1.15", port=29999):
        """
        Initialize Dashboard connection
        
        Args:
            host: UR robot IP address
            port: Dashboard Server port (default 29999)
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect(self):
        """Connect to Dashboard Server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # Set 5 second timeout
            self.socket.connect((self.host, self.port))
            
            # Read welcome message
            welcome_msg = self.socket.recv(1024).decode('utf-8')
            print(f"[Connected] {welcome_msg.strip()}")
            self.connected = True
            return True
            
        except socket.timeout:
            print(f"[ERROR] Connection timeout: {self.host}:{self.port}")
            return False
        except socket.error as e:
            print(f"[ERROR] Connection failed: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from Dashboard Server"""
        if self.socket:
            try:
                # Send quit command before closing
                if self.connected:
                    self.send_command("quit")
                self.socket.close()
                print("[INFO] Dashboard connection closed")
            except:
                pass
            finally:
                self.connected = False
                self.socket = None
                
    def send_command(self, command):
        """
        Send command to Dashboard Server
        
        Args:
            command: Command string to send
            
        Returns:
            Response string, None if failed
        """
        if not self.connected:
            print("[ERROR] Not connected to Dashboard Server")
            return None
            
        try:
            # Send command (add newline)
            self.socket.send((command + "\n").encode('utf-8'))
            
            # Receive response
            response = self.socket.recv(1024).decode('utf-8').strip()
            return response
            
        except socket.error as e:
            print(f"[ERROR] Failed to send command: {e}")
            return None
            
    def power_on(self):
        """Power on the robot"""
        print("[CMD] Powering on robot...")
        response = self.send_command("power on")
        if response:
            print(f"[RESPONSE] {response}")
            return "Powering on" in response or "Powers on" in response
        return False
        
    def power_off(self):
        """Power off the robot"""
        print("[CMD] Powering off robot...")
        response = self.send_command("power off")
        if response:
            print(f"[RESPONSE] {response}")
            return "Powering off" in response or "Powers off" in response
        return False
        
    def brake_release(self):
        """Release the brake"""
        print("[CMD] Releasing brake...")
        response = self.send_command("brake release")
        if response:
            print(f"[RESPONSE] {response}")
            return "Brake releasing" in response or response.startswith("Brake")
        return False
        
    def play(self):
        """Start the current program"""
        print("[CMD] Starting current program...")
        response = self.send_command("play")
        if response:
            print(f"[RESPONSE] {response}")
            return "Starting program" in response or response.startswith("Starting")
        return False
        
    def stop(self):
        """Stop the current program"""
        print("[CMD] Stopping current program...")
        response = self.send_command("stop")
        if response:
            print(f"[RESPONSE] {response}")
            return "Stopped" in response or response.startswith("Stopped")
        return False
        
    def pause(self):
        """Pause the program"""
        print("[CMD] Pausing program...")
        response = self.send_command("pause")
        if response:
            print(f"[RESPONSE] {response}")
            return "Pausing" in response or response.startswith("Pausing")
        return False
        
    def get_robot_mode(self):
        """Get robot mode"""
        response = self.send_command("robotmode")
        if response:
            print(f"[ROBOT MODE] {response}")
            return response
        return None
        
    def get_program_state(self):
        """Get program state"""
        response = self.send_command("programState")
        if response:
            print(f"[PROGRAM STATE] {response}")
            return response
        return None
        
    def is_program_running(self):
        """Check if program is running"""
        response = self.send_command("running")
        if response:
            print(f"[RUNNING STATUS] {response}")
            return response
        return None
        
    def unlock_protective_stop(self):
        """Unlock protective stop"""
        print("[CMD] Unlocking protective stop...")
        response = self.send_command("unlock protective stop")
        if response:
            print(f"[RESPONSE] {response}")
            return True
        return False
        
    def close_popup(self):
        """Close popup window"""
        print("[CMD] Closing popup...")
        response = self.send_command("close popup")
        if response:
            print(f"[RESPONSE] {response}")
            return True
        return False
                
    def clear_operational_mode(self):
        """Clear operational mode"""
        print("[CMD] Clearing operational mode...")
        response = self.send_command("clear operational mode")
        if response:
            print(f"[RESPONSE] {response}")
            return True
        return False
        
    def shutdown(self):
        """Shutdown the robot"""
        print("[CMD] Shutting down robot...")
        response = self.send_command("shutdown")
        if response:
            print(f"[RESPONSE] {response}")
            return True
        return False


def print_menu():
    """Print control menu"""
    print("\n" + "="*50)
    print("UR15 Robot Dashboard Control Menu")
    print("="*50)
    print("1. Power On")
    print("2. Brake Release")
    print("3. Power Off")
    print("4. Play (start current program)")
    print("5. Stop (stop current program)")
    print("6. Pause")
    print("7. Get Robot Mode")
    print("8. Get Program State")
    print("9. Check Running Status")
    print("10. Unlock Protective Stop")
    print("11. Close Popup")
    print("12. Clear Operational Mode")
    print("13. Shutdown")
    print("0. Exit")
    print("="*50)


def main():
    """Main function"""
    # UR robot configuration
    HOST = "192.168.1.15"   # UR robot IP address
    PORT = 29999            # Dashboard Server port
    
    # Create Dashboard control object
    dashboard = URDashboardControl(HOST, PORT)
    
    # Connect to Dashboard Server
    if not dashboard.connect():
        print("[ERROR] Cannot connect to Dashboard Server, exiting...")
        return
    
    try:
        # Main loop
        while True:
            print_menu()
            choice = input("\nPlease select operation (0-15): ").strip()
            
            if choice == '1':
                dashboard.power_on()
            elif choice == '2':
                dashboard.brake_release()
            elif choice == '3':
                dashboard.power_off()
            elif choice == '4':
                dashboard.play()
            elif choice == '5':
                dashboard.stop()
            elif choice == '6':
                dashboard.pause()
            elif choice == '7':
                dashboard.get_robot_mode()
            elif choice == '8':
                dashboard.get_program_state()
            elif choice == '9':
                dashboard.is_program_running()
            elif choice == '10':
                dashboard.unlock_protective_stop()
            elif choice == '11':
                dashboard.close_popup()
            elif choice == '12':
                dashboard.clear_operational_mode()
            elif choice == '13':
                dashboard.shutdown()
            elif choice == '0':
                print("\n[INFO] Exiting program...")
                break
            else:
                print("\n[ERROR] Invalid choice, please try again")
            
            time.sleep(0.5)  # Short delay to avoid too rapid operations
            
    except KeyboardInterrupt:
        print("\n\n[INFO] User interrupted")
    finally:
        # Disconnect
        dashboard.disconnect()
        print("[INFO] Program exited")


if __name__ == "__main__":
    main()
