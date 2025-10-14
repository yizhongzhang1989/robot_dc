#!/usr/bin/env python3
"""
Test UDP data reception frequency from the robot.
This script measures the actual frequency of UDP packets on port 5566.
"""

import socket
import json
import time
import threading
from collections import deque
import statistics

class UDPFrequencyTester:
    def __init__(self, port=5566):
        self.port = port
        self.timestamps = deque(maxlen=1000)  # Keep last 1000 timestamps
        self.packet_count = 0
        self.running = False
        
    def start_monitoring(self):
        """Start UDP monitoring in a separate thread"""
        self.running = True
        thread = threading.Thread(target=self._udp_receiver, daemon=True)
        thread.start()
        return thread
        
    def _udp_receiver(self):
        """UDP receiver thread"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('0.0.0.0', self.port))
            print(f"UDP frequency tester started on port {self.port}")
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(4096)
                    current_time = time.time()
                    
                    # Record timestamp
                    self.timestamps.append(current_time)
                    self.packet_count += 1
                    
                    # Parse and display data occasionally
                    if self.packet_count % 100 == 0:
                        try:
                            message = data.decode('utf-8').strip()
                            robot_data = json.loads(message)
                            print(f"Sample packet #{self.packet_count}:")
                            print(f"  RobotTcpPos: {robot_data.get('RobotTcpPos', 'N/A')}")
                            print(f"  FTSensorData: {robot_data.get('FTSensorData', 'N/A')}")
                            self._print_frequency_stats()
                            print()
                        except (json.JSONDecodeError, UnicodeDecodeError):
                            print(f"Invalid data received: {data[:50]}")
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Error receiving UDP data: {e}")
                    
        except Exception as e:
            print(f"Failed to start UDP receiver: {e}")
        finally:
            sock.close()
            
    def _print_frequency_stats(self):
        """Calculate and print frequency statistics"""
        if len(self.timestamps) < 2:
            return
            
        # Calculate intervals between packets
        intervals = []
        timestamps_list = list(self.timestamps)
        for i in range(1, len(timestamps_list)):
            interval = timestamps_list[i] - timestamps_list[i-1]
            intervals.append(interval)
        
        if intervals:
            avg_interval = statistics.mean(intervals)
            frequency = 1.0 / avg_interval if avg_interval > 0 else 0
            min_interval = min(intervals)
            max_interval = max(intervals)
            max_freq = 1.0 / min_interval if min_interval > 0 else 0
            min_freq = 1.0 / max_interval if max_interval > 0 else 0
            
            print(f"  Frequency Stats (last {len(intervals)} packets):")
            print(f"    Average: {frequency:.2f} Hz")
            print(f"    Range: {min_freq:.2f} - {max_freq:.2f} Hz")
            print(f"    Interval: {avg_interval*1000:.2f} ± {statistics.stdev(intervals)*1000:.2f} ms")
    
    def get_current_frequency(self):
        """Get current average frequency"""
        if len(self.timestamps) < 2:
            return 0
        
        recent_timestamps = list(self.timestamps)[-100:]  # Last 100 packets
        if len(recent_timestamps) < 2:
            return 0
            
        intervals = []
        for i in range(1, len(recent_timestamps)):
            intervals.append(recent_timestamps[i] - recent_timestamps[i-1])
        
        avg_interval = statistics.mean(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0
    
    def stop(self):
        """Stop monitoring"""
        self.running = False

def main():
    print("UDP Frequency Tester for Robot Data")
    print("===================================")
    print("This will test the actual frequency of UDP packets from the robot.")
    print("Press Ctrl+C to stop.\n")
    
    tester = UDPFrequencyTester(port=5566)
    
    try:
        # Start monitoring
        thread = tester.start_monitoring()
        
        # Main monitoring loop
        start_time = time.time()
        while True:
            time.sleep(5)  # Update every 5 seconds
            
            elapsed = time.time() - start_time
            current_freq = tester.get_current_frequency()
            
            print(f"[{elapsed:.1f}s] Total packets: {tester.packet_count}, "
                  f"Current frequency: {current_freq:.2f} Hz")
            
            # Check if we can achieve 30Hz
            if current_freq > 0:
                if current_freq >= 30:
                    print("✅ 30Hz monitoring is ACHIEVABLE with UDP!")
                elif current_freq >= 20:
                    print("⚠️  30Hz might be possible with optimization")
                else:
                    print("❌ 30Hz monitoring not possible with current UDP frequency")
            
    except KeyboardInterrupt:
        print("\nStopping UDP frequency test...")
        tester.stop()
        
        # Final statistics
        print("\nFinal Statistics:")
        print(f"Total packets received: {tester.packet_count}")
        if tester.packet_count > 0:
            total_time = time.time() - start_time
            overall_freq = tester.packet_count / total_time
            print(f"Overall average frequency: {overall_freq:.2f} Hz")
            tester._print_frequency_stats()

if __name__ == "__main__":
    main()
