import serial
import sys
import glob

def list_serial_ports():
    """ Lists serial port names (cross-platform) """
    if sys.platform.startswith('win'):
        ports = [f'COM{i + 1}' for i in range(256)]
    elif sys.platform.startswith('linux'):
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    else:
        raise EnvironmentError('Unsupported platform')
    
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def send_serial_data(port, baudrate=9600, message="Hello"):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            ser.write(message.encode())
            print(f"Sent '{message}' to {port}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    ports = list_serial_ports()
    print("Available ports:", ports)
    
    if ports:
        send_serial_data(ports[0])  # Sends to the first available port
    else:
        print("No serial ports found.")
