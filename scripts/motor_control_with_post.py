import requests

url = 'http://localhost:8000/api/move'  # or replace with your IP
payload = {
    "motor": 1,
    "direction": "left"
}

response = requests.post(url, json=payload)
print(response.status_code)
print(response.json())
