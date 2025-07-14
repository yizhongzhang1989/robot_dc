import cv2
import threading
import subprocess
import numpy as np
import json

MAX_WINDOW_WIDTH = 800

def get_stream_resolution(url):
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "json",
        url
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    info = json.loads(result.stdout)
    w = info["streams"][0]["width"]
    h = info["streams"][0]["height"]
    return w, h

class RTSPStream:
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.width, self.height = get_stream_resolution(self.url)
        print(f"[{self.name}] Stream resolution: {self.width}x{self.height}")

        self.proc = subprocess.Popen(
            [
                "ffmpeg",
                "-rtsp_transport", "tcp",
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-an",  # no audio
                "-i", self.url,
                "-f", "rawvideo",
                "-pix_fmt", "bgr24",
                "-"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )

        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        frame_size = self.width * self.height * 3
        while self.running:
            raw = self.proc.stdout.read(frame_size)
            if len(raw) != frame_size:
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
            with self.lock:
                self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.proc.kill()
        self.thread.join()

def resize_frame(frame, max_width):
    h, w = frame.shape[:2]
    if w > max_width:
        scale = max_width / w
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return frame

def main():
    high_url = "rtsp://admin:123456@192.168.1.100/stream0"
    low_url  = "rtsp://admin:123456@192.168.1.101/stream0"

    stream_high = RTSPStream("High", high_url)
    stream_low  = RTSPStream("Low",  low_url)

    try:
        while True:
            frame_high = stream_high.get_frame()
            frame_low = stream_low.get_frame()

            if frame_high is not None:
                cv2.imshow("High Stream", resize_frame(frame_high, MAX_WINDOW_WIDTH))

            if frame_low is not None:
                cv2.imshow("Low Stream", resize_frame(frame_low, MAX_WINDOW_WIDTH))

            if cv2.waitKey(1) == 27:  # ESC key
                break

    finally:
        stream_high.stop()
        stream_low.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
