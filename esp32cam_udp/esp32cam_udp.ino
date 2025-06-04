#include "esp_camera.h"
#include <WiFi.h>

// ====== Настройки WiFi и UDP =======
const char* ssid = "";      // WiFi network name
const char* password = "";  // WiFi network password 
const char* udp_host = "";  // Client PC IP
const uint16_t udp_port = 5005;

#define CAMERA_FRAME_SIZE    FRAMESIZE_VGA
#define CAMERA_JPEG_QUALITY  12
#define CAMERA_QUEUE_SIZE    2

#define DEBUG_INTERVAL_MS    2000  // период дебага

// ====== Camera pin map for AI-Thinker ESP32-CAM ======
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

// Структура кадра
typedef struct {
  uint8_t* buf;
  size_t len;
  uint32_t id;
  uint32_t t_capture;       // захват начат
  uint32_t t_got_fb;        // получен fb
  uint32_t t_malloc_start;  // malloc start
  uint32_t t_malloc_finish; // malloc finish
  uint32_t t_memcpy_start;  // memcpy start
  uint32_t t_memcpy_finish; // memcpy finish
  uint32_t t_ready;         // готов к отправке
  uint32_t t_send_start;    // начало отправки
  uint32_t t_send_end;      // конец отправки
  uint32_t t_free_start;    // free start
  uint32_t t_free_finish;   // free finish
  uint32_t t_enqueue;       // положен в очередь
  uint32_t t_dequeue;       // извлечён из очереди
} jpg_frame_t;

// ====== Счётчики и дебаг-переменные ======
volatile uint32_t cam_frames = 0;
volatile uint32_t udp_frames = 0;
volatile uint64_t cam_time_sum = 0;
volatile uint64_t fb_time_sum = 0;
volatile uint64_t malloc_time_sum = 0;
volatile uint64_t memcpy_time_sum = 0;
volatile uint64_t enqueue_time_sum = 0;
volatile uint64_t dequeue_time_sum = 0;
volatile uint64_t send_time_sum = 0;
volatile uint64_t free_time_sum = 0;
volatile uint64_t total_cycle_time_sum = 0;
volatile uint64_t queue_wait_sum = 0;

// ====== Ядро 0: захват + препроцессинг ======
void camera_and_preproc_task(void* arg) {
  camera_fb_t* fb = nullptr;
  jpg_frame_t* jpg = nullptr;
  uint32_t frame_id = 0;
  uint32_t t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;

  while (true) {
    t0 = millis(); // старт обработки кадра

    // Захват
    fb = nullptr;
    t1 = millis();
    fb = esp_camera_fb_get();
    t2 = millis();

    if (fb && fb->format == PIXFORMAT_JPEG) {
      jpg = (jpg_frame_t*)malloc(sizeof(jpg_frame_t));
      jpg->id = frame_id++;
      jpg->t_capture = t0;
      jpg->t_got_fb = t2;

      jpg->len = fb->len;

      jpg->t_malloc_start = millis();
      jpg->buf = (uint8_t*)malloc(jpg->len);
      jpg->t_malloc_finish = millis();

      jpg->t_memcpy_start = millis();
      memcpy(jpg->buf, fb->buf, jpg->len);
      jpg->t_memcpy_finish = millis();

      esp_camera_fb_return(fb);

      jpg->t_ready = millis();

      // Положить в очередь
      t3 = millis();
      bool queue_ok = xQueueSend(frame_queue, &jpg, 0) == pdTRUE;
      t4 = millis();
      jpg->t_enqueue = t4;
      if (!queue_ok) {
        jpg->t_free_start = millis();
        free(jpg->buf); free(jpg);
        jpg->t_free_finish = millis();
      }

      // Дебаг: логируем задержки
      cam_frames++;
      cam_time_sum      += (t1 - t0); // подготовка к захвату
      fb_time_sum       += (t2 - t1); // время esp_camera_fb_get
      malloc_time_sum   += (jpg->t_malloc_finish - jpg->t_malloc_start);
      memcpy_time_sum   += (jpg->t_memcpy_finish - jpg->t_memcpy_start);
      enqueue_time_sum  += (t4 - t3);
      total_cycle_time_sum += (t4 - t0);
    }
    // Минимальная задержка для многозадачности
    taskYIELD();
  }
}

// ====== Ядро 1: отправка UDP ======
void udp_task(void* arg) {
  jpg_frame_t* jpg = nullptr;
  const uint16_t max_data = 1400;
  uint32_t t0, t1, t2, t3;
  while (true) {
    t0 = millis();
    if (xQueueReceive(frame_queue, &jpg, portMAX_DELAY) == pdTRUE) {
      t1 = millis();
      jpg->t_dequeue = t1;

      dequeue_time_sum += (t1 - t0);

      jpg->t_send_start = millis();
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

        udp.beginPacket(udp_host, udp_port);
        udp.write(header, sizeof(header));
        udp.write(jpg->buf + offset, chunk);
        udp.endPacket();

        offset += chunk;
        packet_id++;
        delayMicroseconds(250); // минимальная задержка для стабильности
      }
      jpg->t_send_end = millis();
      send_time_sum += (jpg->t_send_end - jpg->t_send_start);

      jpg->t_free_start = millis();
      free(jpg->buf); free(jpg);
      jpg->t_free_finish = millis();
      free_time_sum += (jpg->t_free_finish - jpg->t_free_start);

      udp_frames++;
    }
    taskYIELD();
  }
}

// ====== Дебаг-логгер на Serial ======
void debug_logger_task(void* arg) {
  uint32_t prev_cam_frames = 0, prev_udp_frames = 0;
  uint64_t prev_cam_sum = 0, prev_fb_sum = 0, prev_malloc_sum = 0, prev_memcpy_sum = 0;
  uint64_t prev_enqueue_sum = 0, prev_dequeue_sum = 0, prev_send_sum = 0, prev_free_sum = 0, prev_cycle_sum = 0;
  uint32_t last_dbg = millis();
  while (true) {
    uint32_t now = millis();
    if (now - last_dbg > DEBUG_INTERVAL_MS) {
      uint32_t camf = cam_frames;
      uint32_t udpf = udp_frames;
      uint64_t camdt = cam_time_sum;
      uint64_t fbdt = fb_time_sum;
      uint64_t mallocdt = malloc_time_sum;
      uint64_t memcpydt = memcpy_time_sum;
      uint64_t enqdt = enqueue_time_sum;
      uint64_t deqdt = dequeue_time_sum;
      uint64_t senddt = send_time_sum;
      uint64_t freedt = free_time_sum;
      uint64_t cycledt = total_cycle_time_sum;
      size_t free_psram = ESP.getFreePsram();

      uint32_t delta_cam = camf - prev_cam_frames ? camf - prev_cam_frames : 1;
      uint32_t delta_udp = udpf - prev_udp_frames ? udpf - prev_udp_frames : 1;

      Serial.printf(
        "[DEBUG] FPS: cam %.2f | udp %.2f | cam_prep %.1fms | fb_get %.1fms | malloc %.1fms | memcpy %.1fms | enqueue %.1fms | dequeue %.1fms | udp_send %.1fms | free %.1fms | cycle %.1fms | PSRAM: %u\n",
        (camf - prev_cam_frames) * 1000.0 / DEBUG_INTERVAL_MS,
        (udpf - prev_udp_frames) * 1000.0 / DEBUG_INTERVAL_MS,
        (camdt - prev_cam_sum) * 1.0 / delta_cam,
        (fbdt - prev_fb_sum) * 1.0 / delta_cam,
        (mallocdt - prev_malloc_sum) * 1.0 / delta_cam,
        (memcpydt - prev_memcpy_sum) * 1.0 / delta_cam,
        (enqdt - prev_enqueue_sum) * 1.0 / delta_cam,
        (deqdt - prev_dequeue_sum) * 1.0 / delta_udp,
        (senddt - prev_send_sum) * 1.0 / delta_udp,
        (freedt - prev_free_sum) * 1.0 / delta_udp,
        (cycledt - prev_cycle_sum) * 1.0 / delta_cam,
        free_psram
      );

      prev_cam_frames = camf;
      prev_udp_frames = udpf;
      prev_cam_sum = camdt;
      prev_fb_sum = fbdt;
      prev_malloc_sum = mallocdt;
      prev_memcpy_sum = memcpydt;
      prev_enqueue_sum = enqdt;
      prev_dequeue_sum = deqdt;
      prev_send_sum = senddt;
      prev_free_sum = freedt;
      prev_cycle_sum = cycledt;
      last_dbg = now;
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

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

  bool camera_init = false;

  while (!camera_init) {
    if (esp_camera_init(&config) != ESP_OK) {
      Serial.println("Camera init failed");
      delay(1000);
    } else {
      camera_init = true;
      Serial.println("Camera init OK");
    }
}
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());

  udp.begin(random(1024, 65535));

  frame_queue = xQueueCreate(CAMERA_QUEUE_SIZE, sizeof(jpg_frame_t*));

  // Захват+препроцессинг на ядре 0, отправка на ядре 1, дебаг-логгер на ядре 1
  xTaskCreatePinnedToCore(camera_and_preproc_task, "camera_preproc", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(udp_task, "udp_task", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(debug_logger_task, "debug_log", 6144, NULL, 1, NULL, 1);
}

void loop() {
  delay(1000);
}