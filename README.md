# Drone Project

A drone camera system using ESP32-CAM with MJPEG streaming and telemetry capabilities.

## Project Structure

### 📁 **ESP32-CAM_MJPEG2SD**
Arduino firmware for ESP32-CAM that captures video frames to SD card with MJPEG format and various connectivity options (WiFi, RTSP, MQTT, WebDAV, etc.).

**Setup:** See [esp32_READme.md](esp32_READme.md) in folder of esp32cam-stream for detailed configuration and flashing instructions.

### 📁 **esp32cam-stream**
Python-based streaming system for real-time camera feed transmission via WebSocket.

**Quick Start:**
1. Install dependencies: `pip install -r requirements.txt`
2. Configure WiFi credentials and server host in Python files
3. Run receiver: `python receive_stream.py`
4. Run sender: `python send_image_stream.py`

### 📁 **tests**
Test utilities for system validation:
- `gyro_acc.py` - Gyroscope/accelerometer testing
- `pid_simulator.py` - PID control simulation
- `prop_check.py` - Property validation tests

## Getting Started

1. **Setup ESP32-CAM:** Follow instructions in [esp32_READme.md](esp32_READme.md)
2. **Configure Network:** Set WiFi SSID, password, and static IP
3. **Start Streaming:** Use esp32cam-stream to receive live camera feed
4. **Run Tests:** Validate system components with test suite

## Requirements

- Python 3.x (for streaming and tests)
- Arduino IDE (for ESP32-CAM firmware)
- ESP32-CAM board
- SD card (for video recording)

For detailed setup, refer to individual folder READMEs.