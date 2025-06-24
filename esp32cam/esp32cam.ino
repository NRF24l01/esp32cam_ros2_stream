#include "esp_camera.h"
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include "config.h"

// ====== Настройки камеры (не трогать) ======
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

#define FRAME_SIZE        FRAMESIZE_VGA
#define JPEG_QUALITY      12

#define LED_BUILTIN_PIN 4

WiFiServer server(80);

// ====== Сериал обработчик =======
void printHelp() {
  Serial.println(F("{\"help\":\"Send: ssid|password, example: mywifi|pass123\"}"));
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

void setup() {
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN_PIN, LOW);
  

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Значения по умолчанию
  camConfig.ssid = "robotx";
  camConfig.password = "78914040";
  camConfig.load();

  // Подключаемся к Wi-Fi
  WiFi.begin(camConfig.ssid.c_str(), camConfig.password.c_str());
  Serial.printf("Connecting to WiFi: %s", camConfig.ssid.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    checkSerialInput();
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("WiFi connected. IP address: %s\n", WiFi.localIP().toString().c_str());

  // Инициализация камеры
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = max(JPEG_QUALITY, 12);
    config.fb_count     = 1;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    delay(5000);
    return;
  }
  Serial.println("Camera init OK");

  server.begin();
  Serial.println("HTTP MJPEG stream ready at http://<IP>/stream");
}

void loop() {
  checkSerialInput();

  WiFiClient client = server.available();
  if (client) {
    Serial.println("Client connected");
    client.printf(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "\r\n"
    );

    while (client.connected()) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) break;

      client.printf(
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %u\r\n\r\n",
        fb->len
      );
      client.write(fb->buf, fb->len);
      client.write("\r\n", 2);

      esp_camera_fb_return(fb);
      delay(100); // ~10 fps
      checkSerialInput();
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}