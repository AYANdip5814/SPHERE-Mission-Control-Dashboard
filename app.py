import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import time
import math
import random
import json
import base64
import os

# Optional hardware imports
try:
    import serial
except ImportError:
    serial = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

class HardwareManager:
    def __init__(self):
        self.ser_arduino = None
        self.ser_esp = None
        self.cap = None
        self.running = True

    def start(self):
        if serial:
            try:
                # FIX 1: Change to 9600 baud for Arduino
                self.ser_arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
                sim.connected["arduino"] = True
                print("Hardware: Connected to Arduino on /dev/ttyACM0 at 9600 baud")
            except Exception as e:
                print(f"Hardware Error: Arduino not found. {e}")

            try:
                # ESP8266 Baud is already 9600
                self.ser_esp = serial.Serial('/dev/ttyAMA0', 9600, timeout=0.1)
                sim.connected["esp8266"] = True
                print("Hardware: Connected to ESP8266 on /dev/ttyAMA0 at 9600 baud")
            except Exception as e:
                print(f"Hardware Error: ESP8266 not found. {e}")

        if cv2:
            try:
                self.cap = cv2.VideoCapture(0)
                if self.cap.isOpened():
                    sim.connected["camera"] = True
                    print("Hardware: Camera initialized.")
            except Exception as e:
                print(f"Hardware Error: Camera init failed. {e}")

        # FIX 5: Use socketio.start_background_task instead of threading.Thread
        socketio.start_background_task(self.serial_reader_loop)
        socketio.start_background_task(self.camera_stream_loop)

    def serial_reader_loop(self):
        while self.running:
            if self.ser_arduino and self.ser_arduino.in_waiting:
                try:
                    line = self.ser_arduino.readline().decode('utf-8').strip()
                    if line.startswith('{') and line.endswith('}'):
                        data = json.loads(line)
                        sim.real_sensors = data
                        if self.ser_esp:
                            self.ser_esp.write((line + "\n").encode())
                # FIX 11: Add proper error logging
                except Exception as e:
                    print(f"Serial reader error: {e}")
            socketio.sleep(0.01)

    def camera_stream_loop(self):
        while self.running:
            if self.cap and self.cap.isOpened():
                try:
                    ret, frame = self.cap.read()
                    if ret:
                        frame = cv2.resize(frame, (320, 240))
                        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                        socketio.emit('camera_frame', {'image': jpg_as_text})
                except Exception as e:
                    print(f"Camera stream error: {e}")
            # FIX 6: Change sleep to 0.2 for Pi CPU
            socketio.sleep(0.2)

    def send_to_arduino(self, cmd_dict):
        if self.ser_arduino:
            try:
                msg = json.dumps(cmd_dict) + "\n"
                self.ser_arduino.write(msg.encode())
            except Exception as e:
                print(f"Send to Arduino error: {e}")

class RobotSim:
    def __init__(self):
        # Physics State
        self.pos = {'x': 0, 'y': 0, 'depth': 0}
        self.vel = {'surge': 0, 'sway': 0, 'heave': 0, 'yaw_rate': 0}
        self.orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.accel = {'ax': 0, 'ay': 0, 'az': 9.81}
        
        # Sensor State
        self.battery = 100.0
        self.temp = 24.0
        self.pressure = 1013.25
        self.pressure_baseline = 1013.25
        self.calibrated = False
        self.depth = 0.0
        self.tds = 320
        self.turbidity = 45
        self.noise_class = "CLEAN"
        self.compass = {"mx": 0, "my": 0, "mz": 0}
        
        # Actuator State
        self.inputs = {'surge': 0, 'yaw': 0, 'heave': 0}
        self.lights = False
        
        # Depth Control (PID-like)
        self.target_depth = None
        self.depth_hold_active = False
        self.kp_depth = 1.0  # Simple P controller for depth
        
        # Environment
        self.sonar_objects = [
            {'angle': 45, 'dist': 120, 'width': 10},
            {'angle': 90, 'dist': 80, 'width': 15},
            {'angle': 135, 'dist': 150, 'width': 8}
        ]
        self.sonar_angle = 0
        self.sonar_dir = 1
        self.sonar_reading = 200
        self.sonar2_reading = 200

        self.last_update = time.time()
        self.last_vel = self.vel.copy()
        
        # Hardware Connection Status
        self.connected = {"arduino": False, "camera": False, "gps": False, "esp8266": False}
        self.real_sensors = {}

    def calibrate_pressure(self, p):
        if p > 0:
            self.pressure_baseline = p
            self.calibrated = True
            print(f"Hardware: Depth calibrated at baseline: {p} hPa")

    def update(self):
        now = time.time()
        dt = now - self.last_update
        if dt < 0.001: return
        self.last_update = now
        
        if self.connected["arduino"] and self.real_sensors:
            p = self.real_sensors.get('pressure', 0)
            if not self.calibrated and p > 0:
                self.calibrate_pressure(p)
            
            if self.calibrated:
                self.depth = max(0, (p - self.pressure_baseline) * 0.1)
            
            self.battery = self.real_sensors.get('battery_pct', self.battery)
            self.temp = self.real_sensors.get('temp', self.temp)
            self.tds = self.real_sensors.get('tds', self.tds)
            self.turbidity = self.real_sensors.get('turbidity', self.turbidity)
            self.noise_class = self.real_sensors.get('noise_class', self.noise_class)
            self.compass = self.real_sensors.get('compass', self.compass)
            
            # IMU Safety
            real_imu = self.real_sensors.get('imu', {})
            self.orientation['roll'] = real_imu.get('roll', self.orientation['roll'])
            self.orientation['pitch'] = real_imu.get('pitch', self.orientation['pitch'])
            self.orientation['yaw'] = real_imu.get('yaw', self.orientation['yaw'])
            
            real_gps = self.real_sensors.get('gps', {})
            if real_gps.get('fix'):
                self.connected["gps"] = True
        else:
            # Simulation logic
            target_surge = self.inputs['surge'] * 2.0
            target_yaw = self.inputs['yaw'] * 30.0
            target_heave = self.inputs['heave'] * 0.5

            self.last_vel = self.vel.copy()
            self.vel['surge'] += (target_surge - self.vel['surge']) * 2.0 * dt
            self.vel['yaw_rate'] += (target_yaw - self.vel['yaw_rate']) * 3.0 * dt
            self.vel['heave'] += (target_heave - self.vel['heave']) * 1.0 * dt

            self.accel = {
                'ax': (self.vel['surge'] - self.last_vel['surge']) / dt,
                'ay': 0,
                'az': (self.vel['heave'] - self.last_vel['heave']) / dt + 9.81
            }

            self.orientation['yaw'] = (self.orientation['yaw'] + self.vel['yaw_rate'] * dt) % 360
            self.pos['depth'] = max(0, self.pos['depth'] + self.vel['heave'] * dt)
            self.depth = self.pos['depth']
            
            rad_yaw = math.radians(self.orientation['yaw'])
            self.pos['x'] += self.vel['surge'] * math.sin(rad_yaw) * dt
            self.pos['y'] += self.vel['surge'] * math.cos(rad_yaw) * dt

            self.temp = 24.0 - (self.pos['depth'] * 0.5) + random.uniform(-0.1, 0.1)
            self.pressure = self.pressure_baseline + (self.pos['depth'] * 10)
            
            sweep_speed = 60
            self.sonar_angle += self.sonar_dir * sweep_speed * dt
            # Fix 12: Sonar angle clamp
            self.sonar_angle = max(0, min(180, self.sonar_angle))
            if self.sonar_angle >= 180 or self.sonar_angle <= 0: self.sonar_dir *= -1
                
            self.sonar_reading = 200
            self.sonar2_reading = 200
            s2_angle = (self.sonar_angle + 30) % 180
            for obj in self.sonar_objects:
                if abs(self.sonar_angle - obj['angle']) < obj['width']: self.sonar_reading = obj['dist']
                if abs(s2_angle - obj['angle']) < obj['width']: self.sonar2_reading = obj['dist']
            
            self.battery = max(0, self.battery - (0.01 * dt))
            
            # Compass Simulation (reflecting yaw)
            rad_yaw = math.radians(self.orientation['yaw'])
            self.compass = {
                "mx": math.cos(rad_yaw) * 25.0,
                "my": math.sin(rad_yaw) * 25.0,
                "mz": -45.0
            }

        # Depth Control (P-Controller)
        if self.depth_hold_active and self.target_depth is not None:
            # error = target - current
            # if error > 0 (too shallow), heave should be negative (dive)
            # if error < 0 (too deep), heave should be positive (surface)
            error = self.target_depth - self.depth
            self.inputs['heave'] = max(-1.0, min(1.0, -error * self.kp_depth))
            
            # Auto-update motors when in depth-hold mode
            # We call handle_motor_cmd logic here or emit to it.
            # Easiest is to update the motors directly.
            self.apply_mixing()

    def apply_mixing(self):
        # Mixing logic from hardware doc:
        surge = self.inputs['surge']
        yaw = self.inputs['yaw']
        heave = self.inputs['heave']
        
        # Surge: -1 to 1 -> 0 to 200
        s_val = max(0, surge * 200)
        # Yaw: -1 to 1
        y_val = yaw * 100
        # Heave: -1 to 1 -> 0 to 50
        h_val = max(0, heave * 50)

        t1 = s_val + h_val
        t2 = (s_val * 0.5) + y_val + h_val
        t3 = (s_val * 0.5) - y_val + h_val
        
        # FIX 3: Add h_val back to reverse thrust
        if surge < 0:
            rev_pwr = abs(surge) * 200
            t1 = 0
            t2 = rev_pwr + h_val
            t3 = rev_pwr + h_val
        
        hw.send_to_arduino({
            "T1": int(max(0, min(255, t1))),
            "T2": int(max(0, min(255, t2))),
            "T3": int(max(0, min(255, t3)))
        })

    def get_state(self):
        real_gps = self.real_sensors.get('gps', {})
        lat = real_gps.get('lat', 11.2416 + self.pos['y'] * 9e-6)
        lon = real_gps.get('lon', 79.5053 + self.pos['x'] * 9e-6)
        fix = real_gps.get('fix', self.connected["arduino"]) # Default to true in sim if connected to backend
        
        return {
            "tds": round(self.tds, 1),
            "turbidity": round(self.turbidity, 1),
            "temp": round(self.temp, 1),
            "pressure": round(self.pressure, 1),
            "depth": round(self.depth, 1),
            "noise_class": self.noise_class,
            "compass": self.compass,
            "sonar": {
                "angle": int(self.sonar_angle),
                "us1": round(self.sonar_reading, 1),
                "us2": round(self.sonar2_reading, 1)
            },
            "imu": {
                "roll": round(self.orientation['roll'], 1),
                "pitch": round(self.orientation['pitch'], 1),
                "yaw": round(self.orientation['yaw'], 1),
                "ax": round(self.accel['ax'], 2),
                "ay": round(self.accel['ay'], 2),
                "az": round(self.accel['az'], 2)
            },
            "gps": {
                "lat": lat, "lon": lon, "fix": fix, "sats": real_gps.get('sats', 7)
            },
            "battery_pct": round(self.battery, 1),
            "detections": self.generate_detections(),
            "connected": self.connected,
        }

    def generate_detections(self):
        if random.random() < 0.01:
            # Scale boxes to 320x240 coordinate space
            x1, y1 = random.randint(20, 200), random.randint(20, 150)
            return [{
                "class": random.choice(["PLASTIC", "BIO", "METAL"]),
                "conf": round(random.uniform(0.7, 0.99), 2),
                "box": [x1, y1, x1 + 50, y1 + 40]
            }]
        return []

# FIX 2: Instantiate hw before sim
hw = HardwareManager()
sim = RobotSim()

def sensor_loop():
    while True:
        sim.update()
        socketio.emit('update', sim.get_state())
        socketio.sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('motor_cmd')
def handle_motor_cmd(json_data):
    try:
        joy = json_data['joystick']
        surge = -joy['left']['y']
        yaw = joy['left']['x']
        heave = -joy['right']['y']
        
        sim.inputs = {'surge': surge, 'yaw': yaw, 'heave': heave}
        
        # FIX 14: Log depth hold disengagement
        if abs(heave) > 0.1 and sim.depth_hold_active:
            sim.depth_hold_active = False
            sim.target_depth = None
            print("Depth hold DISENGAGED by manual input")
            socketio.emit('log', {'msg': "Depth hold DISENGAGED by manual input", 'level': 'warn'})
        
        # If depth hold is not active, apply mixing normally
        if not sim.depth_hold_active:
            sim.apply_mixing()
    except Exception as e:
        print(f"Motor Error: {e}")

@socketio.on('depth_cmd')
def handle_depth_cmd(data):
    # Hover: maintain current depth
    # Dive1: target 1m
    if data == 'hover':
        sim.target_depth = sim.depth
        sim.depth_hold_active = True
        print(f"Depth Hold: Active at {sim.target_depth:.1f}m")
    elif data == 'dive1':
        sim.target_depth = 1.0
        sim.depth_hold_active = True
        print("Depth Hold: Active at 1.0m (Dive 1M)")
    elif data == 'stop':
        sim.depth_hold_active = False
        sim.target_depth = None
        sim.inputs['heave'] = 0
        sim.apply_mixing()

@socketio.on('stop_cmd')
def handle_stop_cmd(data):
    sim.inputs = {'surge': 0, 'yaw': 0, 'heave': 0}
    # TIP120 STOP is 0,0,0
    hw.send_to_arduino({"T1": 0, "T2": 0, "T3": 0, "STOP": True})
    print("EMERGENCY STOP")

@socketio.on('lights_cmd')
def handle_lights_cmd(data):
    sim.lights = not sim.lights
    hw.send_to_arduino({"L": 1 if sim.lights else 0})

@socketio.on('calibrate_depth')
def handle_calibrate_depth(data):
    p = sim.real_sensors.get('pressure', sim.pressure)
    sim.calibrate_pressure(p)

if __name__ == '__main__':
    hw.start()
    # FIX 5: Use socketio.start_background_task
    socketio.start_background_task(sensor_loop)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
