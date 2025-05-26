from flask import Flask, Response, stream_with_context
import cv2
import threading
import time

app = Flask(__name__)

# Shared state
camera = None
frame_lock = threading.Lock()
current_frame = None
capture_thread = None
viewer_count = 0
stop_event = threading.Event()


def start_camera():
    global camera, capture_thread, stop_event
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    stop_event.clear()
    capture_thread = threading.Thread(target=capture_frames)
    capture_thread.start()


def stop_camera():
    global camera, stop_event
    stop_event.set()
    if camera is not None:
        camera.release()
        camera = None


def capture_frames():
    global current_frame
    while not stop_event.is_set():
        success, frame = camera.read()
        if not success:
            continue
        with frame_lock:
            current_frame = frame
        time.sleep(0.03)  # ~30 fps


def generate_frames():
    global viewer_count
    viewer_count += 1
    if viewer_count == 1:
        start_camera()

    try:
        while True:
            with frame_lock:
                if current_frame is None:
                    continue
                _, buffer = cv2.imencode('.jpg', current_frame)
                frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    except GeneratorExit:
        pass
    finally:
        viewer_count -= 1
        if viewer_count == 0:
            stop_camera()


@app.route('/')
def index():
    return '<h1>USB Camera Stream</h1><img src="/video_feed">'

@app.route('/video_feed')
def video_feed():
    return Response(stream_with_context(generate_frames()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8010, threaded=True)
