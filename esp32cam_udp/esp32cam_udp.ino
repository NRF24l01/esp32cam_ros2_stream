#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

// Настройки камеры (не трогать)
#define CAMERA_FRAME_SIZE    FRAMESIZE_VGA
#define CAMERA_JPEG_QUALITY  12
#define CAMERA_QUEUE_SIZE    2

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

WiFiUDP udp;
QueueHandle_t frame_queue;

// Тип кадра
typedef struct {
  uint8_t* buf;
  size_t len;
  uint32_t id;
} jpg_frame_t;

// ====== Сериал обработчик =======
void printHelp() {
  Serial.println(F("{\"help\":\"Send: ssid|password|host|port, example: mywifi|pass123|192.168.1.2|5005\"}"));
}

void checkSerialInput() {
  static String input;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) {
        String err;
        if (camConfig.parseAndSet(input, err)) {
          camConfig.save();
          Serial.println(F("{\"msg\":\"config_saved\",\"reboot\":true}"));
          delay(300);
          ESP.restart();
        } else {
          Serial.println("{\"error\":\"" + err + "\"}");
          printHelp();
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}

// ====== Ядро 0: захват + препроцессинг ======
void camera_and_preproc_task(void* arg) {
  camera_fb_t* fb = nullptr;
  jpg_frame_t* jpg = nullptr;
  static uint32_t frame_id = 0;

  while (true) {
    fb = esp_camera_fb_get();
    if (fb && fb->format == PIXFORMAT_JPEG) {
      jpg = (jpg_frame_t*)malloc(sizeof(jpg_frame_t));
      jpg->id = frame_id++;
      jpg->len = fb->len;
      jpg->buf = (uint8_t*)malloc(jpg->len);
      memcpy(jpg->buf, fb->buf, jpg->len);
      esp_camera_fb_return(fb);

      // Положить в очередь, если не получилось — освободить память и дропнуть кадр
      if (xQueueSend(frame_queue, &jpg, 0) != pdTRUE) {
        free(jpg->buf); free(jpg);
      }
    }
    taskYIELD();
  }
}

// ====== Ядро 1: отправка UDP ======
void udp_task(void* arg) {
  jpg_frame_t* jpg = nullptr;
  const uint16_t max_data = 1400;

  while (true) {
    if (xQueueReceive(frame_queue, &jpg, portMAX_DELAY) == pdTRUE) {
      size_t total = jpg->len;
      size_t offset = 0;
      uint16_t packet_id = 0;
      uint16_t num_packets = (total + max_data - 1) / max_data;

      while (offset < total) {
        uint16_t chunk = (total - offset > max_data) ? max_data : (total - offset);
        uint8_t header[16];
        memcpy(header, &jpg->id, 4);
        memcpy(header + 4, &packet_id, 2);
        memcpy(header + 6, &num_packets, 2);
        uint16_t datalen = chunk;
        memcpy(header + 8, &datalen, 2);
        memcpy(header +10, &total, 4);
        uint16_t reserved = 0;
        memcpy(header +14, &reserved, 2);

        udp.beginPacket(camConfig.udp_host.c_str(), camConfig.udp_port);
        udp.write(header, sizeof(header));
        udp.write(jpg->buf + offset, chunk);
        udp.endPacket();
        
        offset += chunk;
        packet_id++;
        delayMicroseconds(250);
      }
      free(jpg->buf); free(jpg);
    }
    taskYIELD();
  }
}

void setup() {
  Serial.begin(115200);

  // Дефолтные значения
  camConfig.ssid = "robotx";
  camConfig.password = "78914040";
  camConfig.udp_host = "192.168.10.139";
  camConfig.udp_port = 5005;
  camConfig.load();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = (CAMERA_JPEG_QUALITY < 12) ? 12 : CAMERA_JPEG_QUALITY;
    config.fb_count = 1;
  }

  while (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Camera init OK");

  WiFi.begin(camConfig.ssid.c_str(), camConfig.password.c_str());
  Serial.print("{\"msg\":\"wifi_connecting\",\"ssid\":\"" + camConfig.ssid + "\"}");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    checkSerialInput();
  }
  Serial.println("{\"msg\":\"wifi_connected\",\"ip\":\"" + WiFi.localIP().toString() + "\"}");
  Serial.println(String("{\"msg\":\"sending_to\",\"ip\":\"") + camConfig.udp_host + "\"}");

  udp.begin(random(1024, 65535));
  frame_queue = xQueueCreate(CAMERA_QUEUE_SIZE, sizeof(jpg_frame_t*));

  xTaskCreatePinnedToCore(camera_and_preproc_task, "camera_preproc", 8196, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(udp_task, "udp_task", 4096, NULL, 2, NULL, 1);
}

void loop() {
  checkSerialInput();
  delay(100);
}