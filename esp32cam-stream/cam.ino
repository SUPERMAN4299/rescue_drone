/*
  ESP32-CAM  →  WebSocket JPEG Streamer
  ═══════════════════════════════════════
  Captures JPEG frames from OV2640 camera and sends them
  over WebSocket to your PC running receive_stream.py

  Board    : AI Thinker ESP32-CAM
  Library  : ArduinoWebsockets  (by Gil Maimon)
             https://github.com/gilmaimon/ArduinoWebsockets
  
  Install libraries via Arduino IDE:
    Tools → Manage Libraries → search "ArduinoWebsockets" → Install
    Tools → Manage Libraries → search "ESP32"             → Install (by Espressif)

  Board settings in Arduino IDE:
    Board            : AI Thinker ESP32-CAM
    Partition Scheme : Huge APP (3MB No OTA)
    Port             : your COM port
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "wifi_config.h"
#include <ArduinoWebsockets.h>

using namespace websockets;

// ── WiFi Config ───────────────────────────────────────────────────────────────
const char* WIFI_SSID     = WIFI_SSID;      // ← change this
const char* WIFI_PASSWORD = WIFI_PASSWORD;  // ← change this

// ── PC WebSocket Server ───────────────────────────────────────────────────────
// This is your PC's LOCAL IP address (not 127.0.0.1)
// Find it by running: ipconfig  (Windows) or ip a (Linux)
// Example: 192.168.1.105
const char* WS_HOST = "192.168.1.XXX";   // ← change to your PC's IP
const int   WS_PORT = 3001;
const char* WS_PATH = "/";

// ── Camera Pin Config (AI Thinker ESP32-CAM) ──────────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4

WebsocketsClient wsClient;
bool wsConnected = false;

// ── Camera Init ───────────────────────────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;   // send JPEG directly — no re-encode

  // Frame size: QVGA (320×240) for low latency on CPU pipeline
  // Options: FRAMESIZE_96X96 / QQVGA / QVGA / CIF / VGA
  // Use QVGA for your CPU YOLO pipeline (matches 320×240 in test_cam.py)
  config.frame_size   = FRAMESIZE_QVGA;  // 320×240
  config.jpeg_quality = 12;              // 0=best 63=worst; 10-15 is good
  config.fb_count     = 1;              // 1 buffer = always freshest frame
  config.grab_mode    = CAMERA_GRAB_LATEST;  // discard old frames automatically

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[Camera] Init failed: 0x%x\n", err);
    return false;
  }

  // Fine-tune image quality
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 0);      // -2 to 2
  s->set_contrast(s, 0);        // -2 to 2
  s->set_saturation(s, 0);      // -2 to 2
  s->set_sharpness(s, 0);       // -2 to 2
  s->set_whitebal(s, 1);        // auto white balance on
  s->set_awb_gain(s, 1);        // auto white balance gain on
  s->set_exposure_ctrl(s, 1);   // auto exposure on
  s->set_aec2(s, 1);            // AEC DSP on
  s->set_ae_level(s, 0);        // -2 to 2
  s->set_gain_ctrl(s, 1);       // auto gain on
  s->set_agc_gain(s, 0);        // 0-30
  s->set_gainceiling(s, (gainceiling_t)0); // 2x
  s->set_bpc(s, 0);             // black pixel correction off
  s->set_wpc(s, 1);             // white pixel correction on
  s->set_raw_gma(s, 1);         // gamma correction on
  s->set_lenc(s, 1);            // lens correction on
  s->set_hmirror(s, 0);         // horizontal mirror
  s->set_vflip(s, 0);           // vertical flip
  s->set_dcw(s, 1);             // downsize enable

  Serial.println("[Camera] ✅ Initialized");
  return true;
}

// ── WiFi Connect ──────────────────────────────────────────────────────────────
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("[WiFi] ✅ Connected  IP: %s\n", WiFi.localIP().toString().c_str());
}

// ── WebSocket Connect ─────────────────────────────────────────────────────────
void connectWebSocket() {
  Serial.printf("[WS] Connecting to ws://%s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);

  wsClient.onMessage([](WebsocketsMessage msg) {
    // We don't expect messages from server; ignore
  });

  wsClient.onEvent([](WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      Serial.println("[WS] ✅ Connected");
      wsConnected = true;
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      Serial.println("[WS] ❌ Disconnected");
      wsConnected = false;
    } else if (event == WebsocketsEvent::GotPing) {
      wsClient.pong();
    }
  });

  wsConnected = wsClient.connect(WS_HOST, WS_PORT, WS_PATH);
  if (!wsConnected) {
    Serial.println("[WS] Connection failed — will retry in loop");
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[Boot] ESP32-CAM WebSocket Streamer");
  pinMode(FLASH_GPIO_NUM, OUTPUT);

  if (!initCamera()) {
    Serial.println("[Boot] Camera failed — halting");
    while (true) delay(1000);
  }

  connectWiFi();
  connectWebSocket();
}

// ── Main Loop ─────────────────────────────────────────────────────────────────
unsigned long lastFrame    = 0;
unsigned long frameCount   = 0;
const int     FRAME_DELAY  = 66;   // ms between frames = ~15 fps
                                   // lower = faster but may overwhelm WiFi

void loop() {

  digitalWrite(FLASH_GPIO_NUM, HIGH);  // turn on flash LED   // Handle incoming WS messages / pings
  if (wsConnected) {
    wsClient.poll();
  }

  // Reconnect if dropped
  if (!wsConnected) {
    Serial.println("[WS] Reconnecting…");
    delay(2000);
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    connectWebSocket();
    return;
  }

  // Rate limit
  unsigned long now = millis();
  if (now - lastFrame < FRAME_DELAY) return;
  lastFrame = now;

  // Capture frame
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[Camera] Frame capture failed");
    return;
  }

  // Send raw JPEG bytes over WebSocket
  bool ok = wsClient.sendBinary((const char*)fb->buf, fb->len);
  
  if (ok) {
    frameCount++;
    if (frameCount % 100 == 0) {
      Serial.printf("[WS] Sent %lu frames  last=%u bytes\n", frameCount, fb->len);
    }
  } else {
    Serial.println("[WS] Send failed");
    wsConnected = false;
  }

  // IMPORTANT: always return the frame buffer
  esp_camera_fb_return(fb);
}
