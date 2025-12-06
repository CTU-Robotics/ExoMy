#!/usr/bin/python3
import time
import evdev
from evdev import InputDevice, ecodes
import smbus
import threading
from picamera2 import Picamera2
import cv2
from flask import Flask, Response

# ================================
# CAMERA & FLASK SETUP (Script 2)
# ================================
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

# ================================
# ROBOT CONTROL DRIVER & CONFIG (Script 1)
# ================================
class PCA9685:
    __MODE1       = 0x00
    __PRESCALE    = 0xFE
    __LED0_ON_L   = 0x06
    __LED0_ON_H   = 0x07
    __LED0_OFF_L  = 0x08
    __LED0_OFF_H  = 0x09

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def setPWMFreq(self, freq):
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = int(round(prescaleval))
        oldmode = self.read(self.__MODE1)
        self.write(self.__MODE1, (oldmode & 0x7F) | 0x10)
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        base = 4 * channel
        self.write(self.__LED0_ON_L + base, on & 0xFF)
        self.write(self.__LED0_ON_H + base, on >> 8)
        self.write(self.__LED0_OFF_L + base, off & 0xFF)
        self.write(self.__LED0_OFF_H + base, off >> 8)

    def setServoPulse(self, channel, pulse_us):
        pulse = int((pulse_us * 4096) / 20000)
        self.setPWM(channel, 0, pulse)

# Configuration
DEVICE_PATH = "/dev/input/event9"

continuous_channels = list(range(1, 7))   # velocity
position_channels   = list(range(7, 13))  # steering

STOP = 1500
MINP = 1300
MAXP = 1700
MIDDLE = 1500
POS_45 = MIDDLE + int((MAXP - MINP) * 1.0)

# Rotation Tables
reverse_mode1 = [1]*16
reverse_mode2 = [1]*16
reverse_mode3 = [1]*16

# Mode 1
reverse_mode1[1:7] = [-1,-1,-1,1,1,1]
reverse_mode1[7:13] = [1,-1,-1,-1,-1,1]

# Mode 2
reverse_mode2[1:7] = [1,1,1,1,1,1]
reverse_mode2[7:13] = [1,1,-1,1,1,-1]

# Mode 3
reverse_mode3[1:13] = [1]*12
reverse_mode3[1:7] = [-1,-1,-1,1,1,1]

# Steering Ranges
# Mode 1 — Normal drive ±90°
steer_min_mode1 = [0]*16
steer_max_mode1 = [0]*16
for ch in position_channels:
    steer_min_mode1[ch] = 1300
    steer_max_mode1[ch] = 1700

# Mode 2 — Point turn (fixed positions)
steer_min_mode2 = [0]*16
steer_max_mode2 = [0]*16
for ch in position_channels:
    steer_min_mode2[ch] = MINP
    steer_max_mode2[ch] = MAXP

# Mode 3 — Crab mode ±180°
steer_min_mode3 = [0]*16
steer_max_mode3 = [0]*16
for ch in position_channels:
    steer_min_mode3[ch] = 600
    steer_max_mode3[ch] = 2400

# Helpers
def map_value(x, in_min, in_max, out_min, out_max):
    return int(out_min + (float(x - in_min) / (in_max - in_min)) * (out_max - out_min))

def apply_reversal(pulse, ch, rev_table):
    """Apply per-channel reversal around MIDDLE."""
    return MIDDLE + (pulse - MIDDLE) * rev_table[ch]

# ================================
# ROBOT LOGIC LOOP
# ================================
def robot_control_loop():
    print("🎮 Initializing PCA9685...")
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)

    try:
        gamepad = InputDevice(DEVICE_PATH)
    except Exception as e:
        print(f"❌ Cannot open device {DEVICE_PATH}: {e}")
        return

    print("\nModes:")
    print("1️⃣ Normal")
    print("2️⃣ Point turn")
    print("3️⃣ Crab mode")
    print("Press SELECT to switch modes.\n")

    mode = 1
    left_val = 128
    right_val = 128

    try:
        for event in gamepad.read_loop():
            # Mode switch
            if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_SELECT and event.value == 1:
                mode = (mode % 3) + 1
                print(f"🔄 Mode changed to {mode}")

            # Skip non-axis events
            if event.type != ecodes.EV_ABS:
                continue

            # Read sticks
            if event.code == ecodes.ABS_Y:
                left_val = event.value
            elif event.code == ecodes.ABS_Z:
                right_val = event.value

            # Select reversal table and steering range tables
            if mode == 1:
                rev = reverse_mode1
                steer_min = steer_min_mode1
                steer_max = steer_max_mode1
            elif mode == 2:
                rev = reverse_mode2
                steer_min = steer_min_mode2
                steer_max = steer_max_mode2
            else:
                rev = reverse_mode3
                steer_min = steer_min_mode3
                steer_max = steer_max_mode3

            # ===== MODE 1: NORMAL DRIVE =====
            if mode == 1:
                # Velocity servos
                vel_pulse = map_value(left_val, 0, 255, MAXP, MINP)
                for ch in continuous_channels:
                    pwm.setServoPulse(ch, apply_reversal(vel_pulse, ch, rev))

                # Position servos
                for ch in position_channels:
                    if ch in (8,11):
                        pwm.setServoPulse(ch, MIDDLE)
                        continue
                    pos_pulse = map_value(right_val, 0, 255, steer_min[ch], steer_max[ch])
                    pwm.setServoPulse(ch, apply_reversal(pos_pulse, ch, rev))

            # ===== MODE 2: POINT TURN =====
            elif mode == 2:
                # Steering fixed positions
                pwm.setServoPulse(8, MIDDLE)
                pwm.setServoPulse(11, MIDDLE)
                for ch in (7,9,10,12):
                    pwm.setServoPulse(ch, apply_reversal(POS_45, ch, rev))

                # Velocity by RIGHT stick
                vel_pulse = map_value(right_val, 0, 255, MAXP, MINP)
                for ch in continuous_channels:
                    pwm.setServoPulse(ch, apply_reversal(vel_pulse, ch, rev))

            # ===== MODE 3: CRAB MODE =====
            elif mode == 3:
                vel_pulse = map_value(left_val, 0, 255, MAXP, MINP)
                pos_pulse = map_value(right_val, 0, 255, steer_min[ch], steer_max[ch])

                for ch in continuous_channels:
                    pwm.setServoPulse(ch, apply_reversal(vel_pulse, ch, rev))
                for ch in position_channels:
                    pulse = map_value(right_val, 0, 255, steer_min[ch], steer_max[ch])
                    pwm.setServoPulse(ch, apply_reversal(pulse, ch, rev))

    except KeyboardInterrupt:
        print("\n🛑 Stopping all servos...")
        for ch in continuous_channels + position_channels:
            pwm.setServoPulse(ch, STOP)
        print("Done.")

# ================================
# MAIN EXECUTION
# ================================
if __name__ == "__main__":
    # Start the Robot Control loop in a separate thread
    robot_thread = threading.Thread(target=robot_control_loop)
    robot_thread.daemon = True # Thread dies when main program exits
    robot_thread.start()

    # Start the Flask Web Server (this blocks the main thread)
    print("📷 Starting Camera Stream on port 8080...")
    app.run(host='0.0.0.0', port=8080)