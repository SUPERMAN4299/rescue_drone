# 🚁 ESP32-CAM Drone Vision System

A comprehensive embedded vision system combining ESP32-CAM hardware with real-time YOLOv8 AI analysis, WebSocket streaming, and advanced sensor fusion capabilities. Perfect for robotics, FPV drones, security systems, and autonomous vehicle applications.

---

## ✨ Key Features

### 🎥 **Hardware & Capture**
- Multiple camera sensors (OV2640, OV3660, OV5640)
- MJPEG2SD firmware with SD card recording & motion detection
- SVGA (800×600) @ 15fps or QVGA (320×240) @ 30fps streaming
- Audio recording & RTSP streaming

### 🤖 **AI & Computer Vision**
- YOLOv8 real-time object detection with auto GPU detection
- Human detection & tracking with unique ID assignment
- GPU acceleration (NVIDIA, AMD, Intel Arc)
- Frame-skipping & jitter reduction

### 🌐 **Connectivity & Streaming**
- WebSocket, RTSP, MQTT, WiFi & serial communication
- Home Assistant integration & Telegram alerts
- FTP, WebDAV, HTTPS file transfer
- Web dashboard for configuration

### 🛠️ **Advanced Features**
- **Telemetry recording** & sensor fusion (gyro + accelerometer)
- **Serial communication** with drone flight controllers
- **PID controller simulation** for motion control tuning
- **Automatic GPU detection** with CPU fallback
- **OTA firmware updates**

---

## 📋 System Architecture

The system consists of three interconnected components:

```
┌─────────────────────────────────────────────────────────────┐
│         ESP32-CAM Hardware (Embedded System)                │
│  • MJPEG2SD Firmware  • SD Card Recording  • Sensors        │
└──────────────────┬──────────────────────────────────────────┘
                   │ WebSocket / Serial / RTSP
                   ▼
┌─────────────────────────────────────────────────────────────┐
│      Python Streaming Layer (Real-time Processing)         │
│  • WebSocket Server  • Frame Transmission  • WiFi Config    │
└──────────────────┬──────────────────────────────────────────┘
                   │ Video Stream
                   ▼
┌─────────────────────────────────────────────────────────────┐
│     YOLOv8 AI Analysis Engine (Client-side)                │
│  • Object Detection  • Human Tracking  • GPU Acceleration   │
│  • Bounding Box Drawing  • Telemetry Logging                │
└─────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
drone/
├── analysing.py                    # Main YOLOv8 analysis script
├── analysing_cap.py                # Analyze saved captures
├── launcher.py                     # Run the pipeline
├── test_cam.py                     # Test detection without ESP32
├── yolov8n.pt                      # YOLOv8 Nano model weights
├── LICENSE                         # MIT License
├── esp32cam-stream/                # Python streaming system
│   ├── websocket_camera_stream.ino # WebSocket camera code
│   ├── requirements.txt
│   ├── extract_wifi.py             # Extract Windows WiFi credentials
│   ├── esp32_READme.md             # Setup & flashing guide
│   └── stream/
│       ├── send_image_stream.py
│       └── receive_stream.py
├── ESP32-CAM_MJPEG2SD/             # Firmware & hardware code
└── tests/                          # Testing tools
    ├── gyro_acc.py                 # Sensor fusion simulator
    ├── pid_simulator.py            # PID tuning simulator
    └── prop_check.py               # Property validation
```

---

## 🚀 Quick Start

### Prerequisites

- **Hardware:** ESP32-CAM, microSD card, USB cable
- **Software:** Python 3.8+, Arduino IDE v2.0+, git

### 1️⃣ Hardware Setup (ESP32-CAM)

```bash
git clone https://github.com/SUPERMAN4299/drone
cd drone/ESP32-CAM_MJPEG2SD

# In Arduino IDE:
# 1. Open ESP32-CAM_MJPEG2SD.ino
# 2. Edit appGlobals.h: Select camera (CAMERA_MODEL_AI_THINKER)
# 3. Tools → Board → ESP32 Dev Module
# 4. Tools → Partition Scheme → Minimal SPIFFS
# 5. Connect ESP32-CAM, tie IO0 to GND for flashing
# 6. Upload & press Reset
```

### 2️⃣ Python Environment Setup

```bash
# Create virtual environment
python -m venv venv
venv\Scripts\activate  # Windows
# or: source venv/bin/activate  # Linux/Mac

# Install dependencies
pip install -r esp32cam-stream/requirements.txt
```

### 3️⃣ Configuration

**Auto WiFi Config (Windows):**
```bash
python esp32cam-stream/extract_wifi.py
```

**Manual WiFi Config:**
- Connect to ESP32-CAM AP: `ESP-CAM_MJPEG_...`
- Browser: `192.168.4.1`
- Set SSID, password & static IP

### 4️⃣ Start Streaming & Analysis

**Terminal 1:**
```bash
python esp32cam-stream/stream/receive_stream.py
```

**Terminal 2:**
```bash
python analysing.py
```

---

## 🔧 Configuration Guide

### ESP32-CAM Firmware (appGlobals.h)

```cpp
// Camera Selection (uncomment ONE)
#define CAMERA_MODEL_AI_THINKER        // Standard ESP32-CAM
//#define CAMERA_MODEL_FREENOVE_ESP32S3_CAM  // Better performance

// Features (set true to enable)
#define INCLUDE_MQTT true              // Home Assistant control
#define INCLUDE_TELEGRAM true          // Alert notifications
#define INCLUDE_RTSP true              // Media player streaming
#define INCLUDE_AUDIO true             // Microphone recording
#define INCLUDE_TELEMETRY true         // Sensor logging

// Video Settings
#define FRAMESIZE FRAMESIZE_SVGA       // 800x600
#define JPEG_QUALITY 12                // 0-63 (higher=better)
```

### Python Analysis (analysing.py)

The script auto-detects your GPU:

```
GPU Detection Order:
1. NVIDIA CUDA (via torch.cuda)
2. AMD Radeon (via WMI)
3. Intel Arc/Xe (via WMI)
4. CPU Fallback

Model Selection:
├─ CUDA >= 8GB VRAM  → YOLOv8x (960×960, High accuracy)
├─ CUDA >= 4GB VRAM  → YOLOv8m (640×640, Balanced)
├─ CUDA < 4GB VRAM   → YOLOv8s (640×640, Fast)
└─ CPU               → YOLOv8n (320×320, Nano)
```

### Network Configuration

Edit `esp32cam-stream/websocket_camera_stream.ino`:

```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* websocket_server = "192.168.1.10";  // PC running receive_stream.py
const int websocket_port = 8765;
```

---

## 🛠️ Utility Commands

### Kill Running Processes
```bash
# Stop Python
taskkill /F /IM python.exe

# PowerShell: Remove pycache
Get-ChildItem -Path . -Filter "__pycache__" -Recurse -Directory | Remove-Item -Force -Recurse

# Clear Ultralytics cache
rm "$env:APPDATA\Ultralytics"  # PowerShell
```

### Check Port Usage
```bash
netstat -ano | findstr :5000
netstat -ano | findstr :8765
taskkill /PID <PID> /F
```

## 🧪 Testing

```bash
# PID Controller Tuning
python tests/pid_simulator.py

# Sensor Fusion Simulator
python tests/gyro_acc.py

# Property Validation
python tests/prop_check.py
```

---

## 🐛 Troubleshooting

**ESP32-CAM Won't Flash:**
- Ensure IO0 connected to GND during flashing
- Try different USB cable (data, not charge-only)
- Update CH340 driver if using clones

**No Video Stream:**
- Verify WiFi connection: `http://<esp32-ip>`
- PC and ESP32-CAM on same network (not VPN)
- Check firewall: allow port 8765 & 8000

**YOLOv8 Slow:**
- Reduce image size: `imgsz=320`
- Use smaller model: `yolov8s.pt`
- Check GPU active: `nvidia-smi`

**GPU Not Detected:**
- Update GPU drivers (NVIDIA/AMD/Intel)
- Check CUDA: `python -c "import torch; print(torch.cuda.is_available())"`
- Restart Python after driver update

**Out of Memory:**
- Use smaller model: `yolov8n.pt`
- Reduce resolution: `imgsz=320`
- Restart Python between runs

---

## 📖 Documentation

- [ESP32-CAM Setup Guide](esp32cam-stream/esp32_READme.md)
- [YOLOv8 Documentation](https://docs.ultralytics.com)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [WebSocket Protocol](https://tools.ietf.org/html/rfc6455)

---

## 🤝 Contributing

Contributions are welcome! Areas for improvement:

- [ ] WebRTC streaming for lower latency
- [ ] Multi-camera support (multiple ESP32s)
- [ ] Custom YOLO model training pipeline
- [ ] Android/iOS mobile app
- [ ] Facial recognition module
- [ ] Gesture-based control
- [ ] Cloud integration (AWS, Azure)
- [ ] Docker containerization

**To contribute:**
```bash
git checkout -b feature/your-feature
git commit -am "Add your feature"
git push origin feature/your-feature
# Create Pull Request
```

---

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **YOLOv8** - Ultralytics object detection framework
- **ESP32-CAM_MJPEG2SD** - Original firmware project
- **OpenCV** - Computer vision library
- **PyTorch** - Deep learning framework

---

## 📞 Support & Contact

For issues and questions:
- 📋 Check [Troubleshooting](#-troubleshooting) section
- 🐛 Open an issue on GitHub
- 💬 Start a discussion for feature requests

**Last Updated:** May 2026  
**Maintainer:** Drone Vision Team
