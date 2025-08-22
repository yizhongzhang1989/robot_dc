#!/usr/bin/env python3
"""
Limited 30Hz Robot Data Monitor
Samples robot data at exactly 30Hz with human-readable timestamps
"""

import socket
import struct
import json
import time
import threading
from datetime import datetime
from dataclasses import dataclass
from typing import Optional

@dataclass
class RobotSample:
    """Single robot data sample"""
    timestamp: float
    readable_time: str
    tcp_position: list  # [x,y,z,rx,ry,rz] in mm/degrees
    joint_positions: list  # Joint angles in radians
    force_torque: list  # [fx,fy,fz,tx,ty,tz] in N/Nm
    source: str  # "UDP" or "TCP"

class Limited30HzMonitor:
    """Monitor robot data at exactly 30Hz"""
    
    def __init__(self):
        # Network settings
        self.udp_port = 5566
        self.tcp_host = "10.75.30.77"
        self.tcp_port = 2001
        
        # Timing control
        self.sample_interval = 1.0 / 30.0  # 30Hz = 33.33ms interval
        
        # Data storage
        self.latest_udp_data = None
        self.latest_tcp_data = None
        self.latest_sample = None
        self.sample_count = 0
        
        # Threading
        self.running = False
        self.udp_active = False
        self.tcp_active = False
        
        # Locks
        self.udp_lock = threading.Lock()
        self.tcp_lock = threading.Lock()
        self.sample_lock = threading.Lock()
        
    def start(self):
        """Start monitoring"""
        self.running = True
        
        # Start data receivers
        udp_thread = threading.Thread(target=self._udp_receiver, daemon=True)
        tcp_thread = threading.Thread(target=self._tcp_receiver, daemon=True)
        
        # Start 30Hz sampler
        sampler_thread = threading.Thread(target=self._sampler_30hz, daemon=True)
        
        # Start display
        display_thread = threading.Thread(target=self._display, daemon=True)
        
        udp_thread.start()
        tcp_thread.start()
        sampler_thread.start()
        display_thread.start()
        
        print("30Hz Monitor started...")
        return [udp_thread, tcp_thread, sampler_thread, display_thread]
    
    def _udp_receiver(self):
        """Receive UDP data (no frequency limit)"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('0.0.0.0', self.udp_port))
            sock.settimeout(1.0)
            self.udp_active = True
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(4096)
                    robot_data = json.loads(data.decode('utf-8').strip())
                    
                    with self.udp_lock:
                        self.latest_udp_data = {
                            'timestamp': time.time(),
                            'tcp_position': robot_data.get('RobotTcpPos', [0]*6),
                            'joint_positions': robot_data.get('RobotAxis', [0]*7),
                            'force_torque': robot_data.get('FTSensorData', [0]*6)
                        }
                        
                except socket.timeout:
                    continue
                except Exception:
                    continue
                    
        except Exception:
            self.udp_active = False
        finally:
            sock.close()
            self.udp_active = False
    
    def _tcp_receiver(self):
        """Receive TCP data (backup source)"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((self.tcp_host, self.tcp_port))
            sock.settimeout(1.0)
            self.tcp_active = True
            
            FRAME_SIZE = 1468
            
            while self.running:
                try:
                    # Read complete frame
                    data = b""
                    while len(data) < FRAME_SIZE:
                        packet = sock.recv(FRAME_SIZE - len(data))
                        if not packet:
                            break
                        data += packet
                    
                    if len(data) == FRAME_SIZE:
                        # Parse data
                        tcp_pos = list(struct.unpack("6f", data[368:392]))
                        joint_pos = list(struct.unpack("7f", data[0:28]))
                        tcp_torque = list(struct.unpack("6f", data[440:464]))
                        
                        with self.tcp_lock:
                            self.latest_tcp_data = {
                                'timestamp': time.time(),
                                'tcp_position': tcp_pos,
                                'joint_positions': joint_pos,
                                'force_torque': tcp_torque
                            }
                
                except socket.timeout:
                    continue
                except Exception:
                    break
                    
        except Exception:
            self.tcp_active = False
        finally:
            try:
                sock.close()
            except:
                pass
            self.tcp_active = False
    
    def _sampler_30hz(self):
        """Sample data at exactly 30Hz"""
        last_sample = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for next sample
            if current_time - last_sample >= self.sample_interval:
                # Get latest data (prefer UDP)
                sample_data = None
                source = ""
                
                with self.udp_lock:
                    if self.latest_udp_data:
                        sample_data = self.latest_udp_data.copy()
                        source = "UDP"
                
                # Fallback to TCP if no UDP data
                if not sample_data:
                    with self.tcp_lock:
                        if self.latest_tcp_data:
                            sample_data = self.latest_tcp_data.copy()
                            source = "TCP"
                
                # Create sample
                if sample_data:
                    readable_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    sample = RobotSample(
                        timestamp=current_time,
                        readable_time=readable_time,
                        tcp_position=sample_data['tcp_position'],
                        joint_positions=sample_data['joint_positions'],
                        force_torque=sample_data['force_torque'],
                        source=source
                    )
                    
                    with self.sample_lock:
                        self.latest_sample = sample
                        self.sample_count += 1
                
                last_sample = current_time
            
            # Small sleep to prevent busy waiting
            time.sleep(0.001)
    
    def _display(self):
        """Display data for each sample"""
        start_time = time.time()
        last_count = 0
        
        while self.running:
            current_time = time.time()
            
            # Check if new sample available
            if self.sample_count > last_count:
                self._show_status(current_time - start_time)
                last_count = self.sample_count
            
            time.sleep(0.01)  # Check every 10ms
    
    def _show_status(self, runtime):
        """Show current status in simple format"""
        with self.sample_lock:
            sample = self.latest_sample
        
        # Simple status line (no screen clearing)
        udp_status = "UDP_OK" if self.udp_active else "UDP_ERR"
        tcp_status = "TCP_OK" if self.tcp_active else "TCP_ERR"
        actual_hz = self.sample_count / runtime if runtime > 0 else 0
        
        # Print status every 50 samples to avoid spam
        if self.sample_count % 50 == 0:
            print(f"STATUS: {udp_status} {tcp_status} | Samples: {self.sample_count} | Rate: {actual_hz:.1f}Hz | Runtime: {runtime:.1f}s")
        
        # Print data in simple format: Time | Position(6) | Force(6) | Source
        if sample:
            pos_str = ' '.join(f'{x:.3f}' for x in sample.tcp_position)
            force_str = ' '.join(f'{x:.3f}' for x in sample.force_torque)
            print(f"{sample.readable_time} | {pos_str} | {force_str} | {sample.source}")
    
    def stop(self):
        """Stop monitoring"""
        self.running = False

def main():
    """Main function"""
    print("30Hz Robot Data Monitor - Simple Format")
    print("======================================")
    print("Format: Time | Position(x y z rx ry rz) | Force(fx fy fz tx ty tz) | Source")
    print("Press Ctrl+C to stop\n")
    
    monitor = Limited30HzMonitor()
    
    try:
        threads = monitor.start()
        
        # Keep running
        while monitor.running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        monitor.stop()
        
        # Wait for threads
        for thread in threads:
            thread.join(timeout=1.0)
        
        print("Stopped.")

if __name__ == "__main__":
    main()
