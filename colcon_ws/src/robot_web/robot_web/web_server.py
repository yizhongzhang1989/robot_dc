from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import os
from ament_index_python.packages import get_package_share_directory

from .web_ros_client import WebROSClient

app = FastAPI()
ros_client = None

web_path = os.path.join(get_package_share_directory('robot_web'), 'web')
STATIC_DIR = os.path.abspath(web_path)
print("Serving web from:", STATIC_DIR, flush=True)


@app.on_event("startup")
def initialize():
    global ros_client
    motor_names = os.environ.get("MOTOR_NAMES", "")
    motor_list = motor_names.split(",") if motor_names else []
    ros_client = WebROSClient(motor_list)

@app.get("/")
def serve_index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))

@app.get("/motors")
def get_motors():
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    return {"motors": ros_client.motor_list}

@app.post("/api/{motor_id}/cmd")
async def send_motor_command(motor_id: str, request: Request):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    data = await request.json()
    command = data.get("command")
    value = data.get("value", None)
    return ros_client.send_command(motor_id, command, value)

@app.get("/api/{motor_id}/status")
def get_motor_status(motor_id: str):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    status = ros_client.get_motor_status(motor_id)
    if status is None:
        return JSONResponse(content={"error": "No status yet"}, status_code=404)
    return status

@app.get("/api/all_status")
def get_all_status():
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    return ros_client.get_all_status()

@app.post("/api/observation/{target}/{action}")
def control_observation(target: str, action: str):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    if action not in ("start", "stop"):
        return JSONResponse(content={"error": "Invalid action"}, status_code=400)
    result = ros_client.control_observation(target, action)
    if "error" in result:
        return JSONResponse(content=result, status_code=400)
    return result

@app.post("/monitor_control")
async def monitor_control(request: Request):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    data = await request.json()
    action = data.get("action")
    if action not in ("start", "stop"):
        return JSONResponse(content={"error": "Invalid action"}, status_code=400)
    # Control all motors/servos
    targets = ["motor1", "motor2", "servo17", "servo18"]
    results = []
    for target in targets:
        result = ros_client.control_observation(target, action)
        results.append(result)
    return {"result": results}

@app.post("/snapshot")
async def take_snapshot():
    """Take snapshots from all cameras using the camera_node services."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.get_snapshot()
    return JSONResponse(content=result)

@app.post("/snapshot/{camera_name}")
async def take_single_snapshot(camera_name: str):
    """Take snapshot from specific camera."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.get_snapshot(camera_name)
    return JSONResponse(content=result)

@app.post("/restart_camera")
async def restart_camera():
    """Restart all camera nodes using the restart services."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.restart_camera_node()
    return JSONResponse(content=result)

@app.post("/restart_camera/{camera_name}")
async def restart_single_camera(camera_name: str):
    """Restart specific camera node."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.restart_camera_node(camera_name)
    return JSONResponse(content=result)

@app.post("/api/platform/cmd")
async def platform_command(request: Request):
    """Send command to platform controller."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    data = await request.json()
    command = data.get("command")
    value = data.get("value", None)
    
    if not command:
        return JSONResponse(content={"error": "Command is required"}, status_code=400)
    
    result = ros_client.send_platform_command(command, value)
    
    if "error" in result:
        return JSONResponse(content=result, status_code=400)
    return JSONResponse(content=result)

@app.post("/api/platform/timed_move")
async def platform_timed_move(request: Request):
    """Send timed movement command to platform controller."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    data = await request.json()
    direction = data.get("direction")
    duration = data.get("duration")
    
    if not direction or duration is None:
        return JSONResponse(content={"error": "Direction and duration are required"}, status_code=400)
    
    # Send the timed movement command
    command = f"timed_{direction}"
    result = ros_client.send_platform_command(command, duration)
    
    if "error" in result:
        return JSONResponse(content=result, status_code=400)
    return JSONResponse(content=result)

@app.get("/stream/{camera_name}")
async def stream_camera(camera_name: str):
    """Stream video from specific camera node."""
    # Map camera names to their streaming ports
    camera_ports = {
        "cam100": 8010,
        "cam101": 8011
    }
    
    if camera_name not in camera_ports:
        return JSONResponse(content={"error": "Unknown camera"}, status_code=404)
    
    port = camera_ports[camera_name]
    stream_url = f"http://localhost:{port}/video_feed"
    
    try:
        import requests
        # Use a shorter timeout to avoid blocking
        response = requests.get(stream_url, stream=True, timeout=3)
        
        def generate():
            try:
                for chunk in response.iter_content(chunk_size=1024):
                    if chunk:
                        yield chunk
            except Exception as e:
                # Log error but don't crash the generator
                print(f"Stream error for {camera_name}: {e}")
                return
        
        return StreamingResponse(
            generate(),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )
    except requests.exceptions.ConnectionError:
        # Return a placeholder image for connection errors
        return _generate_error_stream(camera_name, "Connection failed")
    except requests.exceptions.Timeout:
        # Return a placeholder image for timeouts
        return _generate_error_stream(camera_name, "Stream timeout")
    except Exception as e:
        # Return a placeholder image for other errors
        return _generate_error_stream(camera_name, f"Stream error: {str(e)}")

def _generate_error_stream(camera_name: str, error_message: str):
    """Generate an error image stream when camera is not available."""
    import cv2
    import numpy as np
    
    def generate_error_frames():
        # Create error image
        error_img = np.zeros((240, 320, 3), dtype=np.uint8)
        error_img[:] = (50, 50, 50)  # Dark gray background
        
        # Add text
        cv2.putText(error_img, f"Camera {camera_name}", (20, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(error_img, "Not Available", (20, 125), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(error_img, error_message, (20, 170), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Encode as JPEG
        _, buffer = cv2.imencode('.jpg', error_img)
        frame_bytes = buffer.tobytes()
        
        # Return as MJPEG stream
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            import time
            time.sleep(1)  # Update every second
    
    return StreamingResponse(
        generate_error_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)