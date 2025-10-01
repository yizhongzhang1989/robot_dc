import requests
import time

WIN_HOST = "192.168.1.20"   # IP address of the industrial PC
PORT = 5000                 # port of monitor of ForceMaster.exe
BASE_URL = f"http://{WIN_HOST}:{PORT}"


def get_FTCexe_status():
    """get the status of ForceMaster.exe"""
    try:
        resp = requests.get(f"{BASE_URL}/status", timeout=5)
        return resp.json()
    except Exception as e:
        return {"error": str(e)}


def start_FTCexe():
    """start the ForceMaster.exe program by start.bat"""
    try:
        resp = requests.post(f"{BASE_URL}/start", timeout=5)
        return resp.json()
    except Exception as e:
        return {"error": str(e)}


def stop_FTCexe():
    """stop the ForceMaster.exe program by close.bat"""
    try:
        resp = requests.post(f"{BASE_URL}/stop", timeout=5)
        return resp.json()
    except Exception as e:
        return {"error": str(e)}


if __name__ == "__main__":

    status = get_FTCexe_status()
    print(f"Status: {status['status']}")
    time.sleep(1)

   
