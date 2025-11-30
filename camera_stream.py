from picamera2 import Picamera2
import cv2
from flask import Flask, Response

app = Flask(__name__)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1920, 1080)}))
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        # Correct the color order
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return '<html><body><img src="/video"></body></html>'

@app.route('/video')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080)
