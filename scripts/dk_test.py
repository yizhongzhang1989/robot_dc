#!/usr/bin/env python3  
"""  
This script reads UDP datagrams from a robot.  
Each datagram is assumed to contain a JSON string like:  
  
  {"RobotTcpPos":[-98.85,658.5,500.85,-3.81,0.13,-170.38],  
   "RobotAxis":[1.51,0.36,1.05,0,-1.57,0],  
   "RobotTrack":0,  
   "FTSensorData":[56.76,12.78,31.4,0.89,-2.07,0.33],  
   "FTTarget":[0,0,0,0,0,0]}  
   
A timestamp is added to record when the data was received.  
The data is printed to the terminal.  
Additionally, a separate thread (udp_logger) opens UDP port 5577 and prints any received messages  
along with the sender's IP address and source port.  
"""  
import socket  
import json  
import time  
import threading  
   
# ----- Global Variables and Settings -----  
HOST = "0.0.0.0"  
PORT_DATA = 5566   # Port used for receiving JSON data from robot
PORT_LOG  = 5577   # Port used for logging incoming messages  
   
# ----- UDP Receiver (Data Acquisition) -----  
def udp_receiver():  
    """  
    Create a UDP socket, receive data, decode JSON, and print to terminal.
    """  
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Allow multiple sockets to bind to the same port
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass  # SO_REUSEPORT not available on this system
    try:  
        sock.bind((HOST, PORT_DATA))  
        print(f"Listening for UDP packets on {HOST}:{PORT_DATA} for data reception")  
    except Exception as e:  
        print("Failed to bind data socket:", e)  
        return  
  
    while True:  
        try:  
            data, addr = sock.recvfrom(4096)  
            message = data.decode("utf-8").strip()  
            # Print the received message to terminal
            print(f"Received UDP data from {addr[0]}:{addr[1]}: {message}")
            # Parse JSON to verify it's valid
            record = json.loads(message)  
        except json.JSONDecodeError as je:  
            print("JSON decode error:", je)  
        except KeyboardInterrupt:  
            break  
        except Exception as e:  
            print("Error while receiving data:", e)  
    sock.close()  
    print("Data socket closed.")  
   
# ----- UDP Logger -----  
def udp_logger():  
    """  
    Create a UDP socket on port 5577 and listen for incoming messages.  
    Prints the sender's IP address, source port, and the received message.  
    """  
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    try:  
        sock.bind((HOST, PORT_LOG))  
        print(f"Listening for UDP packets on {HOST}:{PORT_LOG} for logging")  
    except Exception as e:  
        print("Failed to bind logger socket:", e)  
        return  
  
    while True:  
        try:  
            data, addr = sock.recvfrom(4096)  
            message = data.decode("utf-8", errors="ignore").strip()  
            print(f"Logger: Received data from {addr[0]}:{addr[1]}: {message}")  
        except KeyboardInterrupt:  
            break  
        except Exception as e:  
            print("Logger error:", e)  
    sock.close()  
    print("Logger socket closed.")  
   
# ----- Main Function -----  
def main():  
    # Start the data acquisition thread (port 5566) for data reception.  
    recv_thread = threading.Thread(target=udp_receiver, daemon=True)  
    recv_thread.start()  
  
    # Start the logger thread (port 5577) to print incoming messages.  
    logger_thread = threading.Thread(target=udp_logger, daemon=True)  
    logger_thread.start()  
  
    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")  
   
if __name__ == '__main__':  
    main()  
