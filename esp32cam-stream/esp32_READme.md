# ESP32 Setup Instructions

## Download and Installation

- Download ESP32 by Esiff
- Partition scheme: **Huge APP (3MB No OTA/1MB SPIFFS)** you can change it according to your board.
  - If partition doesn't work, use minimal (under this)

## ESP32-CAM MJPEG2SD Configuration

### Initial Setup
- Check `appGlobal.h` for your version (22-32)
- Add SSID and password in `utils.h`
- Data should be sent to SD card

### Uploading Firmware

1. Connect **IO0 to GND** for flashing mode
2. Press reset button
3. After uploading, disconnect **IO0 from GND**
4. Reset again

### Network Configuration

- Check serial monitor for IP address
- Save changes in access settings
- **Recommended:** SVGA (800x600) 15fps
- Add router IP
- Edit config > WiFi and set a static IP (e.g., 192.168.x.x)

## ESP32CAM-Stream Setup

### Python Configuration

- Open `esp32cam-stream`then `websocket_camera_stream.ino`
- In websocket section, update:
  - SSID
  - Password
- Install dependencies: `pip install -r requirements.txt`

### Running the Stream

1. **First flash the .ino code in the esp32 cam**
2. **First terminal:** Run `python receive_stream.py` (displays image size)
3. **Second terminal:** Run`python send_image_stream.py` (sends images to ESP32)