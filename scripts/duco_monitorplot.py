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
The data is then visualized in an OpenCV window (size 1000×600)  
divided into four quadrants:  
  • Top left:   RobotTcpPos  
  • Bottom left: RobotAxis  
  • Top right:  FTSensorData  
  • Bottom right: FTTarget  
   
Each plot displays six curves (each with a unique color:  
red, green, blue, purple, pink and black) for ~1 second  
of incoming data. Instead of dynamically scaling the vertical range,  
each plot uses a fixed range:  
    RobotTcpPos:   [-180, 180]  
    RobotAxis:     [-pi, pi]  
    FTSensorData:  [-50, 50]  
    FTTarget:      [-20, 20]  
The UDP receiving (for visualization) and the visualization functionalities are done in one thread.  
Additionally, a separate thread (udp_logger) opens UDP port 5577 and prints any received messages  
along with the sender's IP address and source port.  
"""  
import os

# Force Qt to use the XCB platform plugin to avoid missing Wayland plugin errors.
# This must be set before importing cv2, which bundles its own Qt runtime.
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import socket  
import json  
import time  
import threading
import numpy as np  
import math  
   
# ----- Global Variables and Settings -----  
HOST = "0.0.0.0"  
PORT_DATA = 5566   # Port used for receiving JSON data for visualization  
PORT_LOG  = 5577   # Port used for logging incoming messages  
   
# Duration (in seconds) to keep data for plotting.  
PLOT_TIME_PERIOD = 1.0  
   
# Colors for up to 6 curves (B, G, R order for OpenCV):  
curve_colors = [  
    (0, 0, 255),     # red  
    (0, 255, 0),     # green  
    (255, 0, 0),     # blue  
    (128, 0, 128),   # purple  
    (180, 105, 255), # pink-ish (hot pink)  
    (0, 0, 0)        # black  
]  
   
# The shared buffer holds the recent data for four plot types.  
# For each plot we expect 6 channels so we use a list of 6 lists.  
# Each channel list will store tuples: (timestamp, value)  
plot_buffer = {  
    "RobotTcpPos": [ [] for _ in range(6) ],  
    "RobotAxis": [ [] for _ in range(6) ],  
    "FTSensorData": [ [] for _ in range(6) ],  
    "FTTarget": [ [] for _ in range(6) ],  
}  
   
# A lock to protect concurrent access to the plot_buffer.  
buffer_lock = threading.Lock()  
   
# ----- Fixed Plot Ranges -----  
FIXED_RANGES = {  
    "RobotTcpPos": (-180, 180),  
    "RobotAxis": (-math.pi, math.pi),  
    "FTSensorData": (-50, 50),  
    "FTTarget": (-20, 20)  
}  
   
# ----- UDP Receiver (Data Acquisition for Visualization) -----  
def udp_receiver():  
    """  
    Create a UDP socket on port 5566, receive data, decode JSON,  
    add a timestamp, and store the data into plot_buffer.  
    This thread does not print any of the received data.  
    """  
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
    try:  
        sock.bind((HOST, PORT_DATA))  
        print(f"Listening for UDP packets on {HOST}:{PORT_DATA} for data visualization")  
    except Exception as e:  
        print("Failed to bind data socket:", e)  
        return  
  
    while True:  
        try:  
            data, addr = sock.recvfrom(4096)  
            message = data.decode("utf-8").strip()  
            # Do NOT print the received data here  
            record = json.loads(message)  
            now = time.time()  
            with buffer_lock:  
                for key in plot_buffer:  
                    if key in record:  
                        values = record[key]  
                        for i in range(min(6, len(values))):  
                            plot_buffer[key][i].append( (now, float(values[i])) )  
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
   
# ----- Visualization Thread -----  
def visualization_thread():  
    """  
    Create an OpenCV window divided into four subplots and update the plots every 30ms.  
    """  
    window_name = "Robot Data"  
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  
    cv2.resizeWindow(window_name, 1000, 600)  
  
    while True:  
        current_time = time.time()  
        local_data = {}  
        with buffer_lock:  
            for key in plot_buffer:  
                local_data[key] = []  
                for ch in plot_buffer[key]:  
                    filtered = [(t, v) for (t, v) in ch if t >= current_time - PLOT_TIME_PERIOD]  
                    ch[:] = filtered  
                    local_data[key].append(list(filtered))  
  
        canvas = np.ones((600, 1000, 3), dtype=np.uint8) * 255  
  
        draw_plot(canvas, local_data.get("RobotTcpPos"), (0, 0, 500, 300), "RobotTcpPos", current_time)  
        draw_plot(canvas, local_data.get("RobotAxis"), (0, 300, 500, 300), "RobotAxis", current_time)  
        draw_plot(canvas, local_data.get("FTSensorData"), (500, 0, 500, 300), "FTSensorData", current_time)  
        draw_plot(canvas, local_data.get("FTTarget"), (500, 300, 500, 300), "FTTarget", current_time)  
  
        cv2.imshow(window_name, canvas)  
        key = cv2.waitKey(30)  
        if key == 27:   # ESC key breaks out of the loop  
            break  
  
    cv2.destroyAllWindows()  
   
def draw_plot(canvas, channels, region, title, current_time):  
    """  
    Draw one subplot into the canvas.  
      canvas: overall drawing area (numpy array image)  
      channels: a list (of 6 channels) of data points [(timestamp, value), ...]  
      region: a tuple (x_offset, y_offset, width, height) for this plot  
      title: text title to draw in the region  
      current_time: current time for mapping timestamps to x coordinates  
    """  
    if channels is None:  
        return  
  
    x_offset, y_offset, width, height = region  
    cv2.rectangle(canvas, (x_offset, y_offset), (x_offset + width, y_offset + height), (0, 0, 0), 1)  
    cv2.putText(canvas, title, (x_offset + 5, y_offset + 20),  
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)  
  
    if title in FIXED_RANGES:  
        v_min, v_max = FIXED_RANGES[title]  
    else:  
        v_min, v_max = (0, 1)  
  
    data_found = False  
    for i, ch in enumerate(channels):  
        if len(ch) < 2:  
            continue  
  
        data_found = True  
        ch_sorted = sorted(ch, key=lambda tup: tup[0])  
        pts = []  
        for (t, v) in ch_sorted:  
            rel_t = (t - (current_time - PLOT_TIME_PERIOD)) / PLOT_TIME_PERIOD  
            if rel_t < 0 or rel_t > 1:  
                continue  
            px = int(rel_t * width)  
            py = int((v_max - v) / (v_max - v_min) * height)  
            pts.append((px, py))  
  
        if len(pts) >= 2:  
            pts_arr = np.array(pts, np.int32).reshape((-1, 1, 2))  
            cv2.polylines(canvas[y_offset:y_offset+height, x_offset:x_offset+width], [pts_arr], False, curve_colors[i], 1)  
  
    if not data_found:  
        cv2.putText(canvas, "No data", (x_offset + width // 2 - 40, y_offset + height // 2),  
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)  
  
    cv2.putText(canvas, f"{v_max:.2f}", (x_offset + width - 70, y_offset + 20),  
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)  
    cv2.putText(canvas, f"{v_min:.2f}", (x_offset + width - 70, y_offset + height - 5),  
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)  
   
# ----- Main Function -----  
def main():  
    # Start the data acquisition thread (port 5566) for visualization.  
    recv_thread = threading.Thread(target=udp_receiver, daemon=True)  
    recv_thread.start()  
  
    # Start the logger thread (port 5577) to print incoming messages.  
    logger_thread = threading.Thread(target=udp_logger, daemon=True)  
    logger_thread.start()  
  
    # Start the visualization thread (this call blocks until the user exits).  
    visualization_thread()  
  
    print("Exiting.")  
   
if __name__ == '__main__':  
    main()  
