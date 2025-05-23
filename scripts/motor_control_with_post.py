import requests
import time


# Example of sending a POST request to control a motor

print("motor1 start moving velocity mode")
motor1_url = 'http://localhost:8000/api/motor1/cmd'
cmd = {
    "command": "move_vel",
    "value": 100
}
response = requests.post(motor1_url, json=cmd)

# sleep for 5 seconds
time.sleep(5)

print("motor1 stop moving")
motor1_url = 'http://localhost:8000/api/motor1/cmd'
cmd = {
    "command": "stop"
}
response = requests.post(motor1_url, json=cmd)

print("motor2 start moving velocity mode")
motor2_url = 'http://localhost:8000/api/motor2/cmd'
cmd = {
    "command": "move_vel",
    "value": -10
}
response = requests.post(motor2_url, json=cmd)

# sleep for 5 seconds
time.sleep(5)

print("motor2 stop moving")
motor2_url = 'http://localhost:8000/api/motor2/cmd'
cmd = {
    "command": "stop" 
}
response = requests.post(motor2_url, json=cmd)