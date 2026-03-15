# SPHERE Mission Control

A single-file HTML/CSS/JS web application that serves as the mission control dashboard and live game controller for the **SPHERE** underwater robot.

## Features

- **Real-time Telemetry**: 150ms updates for sensor data (TDS, Turbidity, Temp, Pressure, Sonar, IMU, GPS, Compass, Depth).
- **Hardware Integration**: Dedicated Serial support for Arduino sensors and ESP8266 WiFi broadcast.
- **Live Camera Stream**: OpenCV-based real-time video feed with AI overlay.
- **HUD Aesthetic**: Tactical dark-mode UI with Orbitron/Rajdhani fonts and circular instruments.
- **Dual Joystick Controls**: Normalized joystick-to-PWM mapping for precision thruster control.
- **Sonar Radar**: Dual-beam sonar rendering with age-based blip expiry.
- **Advanced Mapping**: GPS trail tracking with dynamic bounding-box auto-scaling.
- **Input Handling**: Support for Keyboard, Mouse/Touch, and Gamepad API (20Hz throttled).
- **Smart Calibration**: One-touch surface pressure calibration for accurate depth sensing.

## Project Structure

- `app.py`: Flask-SocketIO backend with physics simulation.
- `templates/index.html`: Monolithic frontend with all HTML, CSS, and JS.
- `requirements.txt`: Python dependencies.

## Installation & Usage

1. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Run the Server**:
   ```bash
   python app.py
   ```

3. **Access Dashboard**:
   Open [http://localhost:5000](http://localhost:5000) in your browser.

## Standalone Mode

The frontend can also run in standalone mode by opening `templates/index.html` directly in any modern browser. It will automatically enter **Simulation Mode** if the backend server is not detected.
