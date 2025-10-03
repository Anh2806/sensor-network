#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================= SPI riêng cho LoRa (giữ kiểu bạn đang dùng) =================
SPIClass SPI_LORA(1);


// ==== Chân LoRa (theo code của bạn) ====
#define LORA_CS   13
#define LORA_RST  14
#define LORA_DIO0 9
#define LORA_MISO 10 
#define LORA_MOSI 11
#define LORA_SCK 12

// ================== WiFi & MQTT ==================
const char* WIFI_SSID = "TUVE";
const char* WIFI_PASS = "12345678";
const char* MQTT_HOST = "kiettong2004.3utilities.com";
const uint16_t MQTT_PORT = 1883;   // không TLS

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ================== Dữ liệu & Queue ==================
struct SensorData {
  char node;           // 'A' hoặc 'B'
  char nhietDo[16];    // Node A: T
  char doAm[16];       // Node A: H
  char lux[16];        // Node B: LUX
  char soil[16];       // Node B: SOIL
  char rssi[8];
  char snr[8];
};

QueueHandle_t dataQueue;

// ================== Helper ==================
static void wifiConnectBlocking() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("WiFi: connecting to %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\nWiFi: retry...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
}

static void mqttEnsureConnected() {
  if (mqttClient.connected()) return;
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  Serial.printf("MQTT: connecting %s:%u ...\n", MQTT_HOST, MQTT_PORT);
  String clientId = String("esp32-wifi-") + String((uint32_t)ESP.getEfuseMac(), HEX);
  uint32_t t0 = millis();
  while (!mqttClient.connected()) {
    if (mqttClient.connect(clientId.c_str())) break;
    Serial.printf("MQTT connect failed, rc=%d\n", mqttClient.state());
    delay(1000);
    if (millis() - t0 > 15000) {
      Serial.println("MQTT: long retry pause...");
      delay(3000);
      t0 = millis();
    }
  }
  Serial.println("MQTT connected.");
}

// ================== Task LoRa ==================
void TaskLoRa(void *pvParameters) {
  bool toggleMessage = true;
  unsigned long lastSendTime = 0;
  const int interval = 3000;

  
  SPI_LORA.begin(LORA_SCK,LORA_MISO,LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI_LORA);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  Serial.println("Khởi tạo LoRa...");
  if (!LoRa.begin(433E6)) {
    Serial.println("Lỗi khởi tạo LoRa!");
    vTaskDelete(NULL);
  }
  Serial.println("LoRa đã sẵn sàng.");

  for (;;) {
    // Gửi lệnh luân phiên tới Node A/B
    if (millis() - lastSendTime > (unsigned long)interval) {
      String outGoing = toggleMessage ? "NODE-1" : "NODE-2"; 
      LoRa.beginPacket();
      LoRa.print(outGoing);
      LoRa.endPacket();

      Serial.println("Gửi lệnh: " + outGoing);
      toggleMessage = !toggleMessage;
      lastSendTime = millis();
    }

    // Nhận gói LoRa
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      String inComing = "";
      while (LoRa.available()) {
        inComing += (char)LoRa.read();
      }
      inComing.trim();
      Serial.println("Nhận: " + inComing);

      int   rssi = LoRa.packetRssi();
      float snr  = LoRa.packetSnr();
      char buffRSSI[8];  snprintf(buffRSSI, sizeof(buffRSSI), "%d", rssi);
      char buffSNR[8];   snprintf(buffSNR,  sizeof(buffSNR),  "%.2f", snr);

      // ---------- Parse theo format mới ----------
      // Node A: "T:xx H:xx"
      if (inComing.startsWith("T:")) {
        float t=0, h=0;
        // Cho phép khoảng trắng linh hoạt giữa các phần
        // Ví dụ hợp lệ: "T:27.45 H:63.10"
        if (sscanf(inComing.c_str(), "T:%f H:%f", &t, &h) == 2) {
          SensorData dataA; memset(&dataA, 0, sizeof(SensorData));
          dataA.node = 'A';
          snprintf(dataA.nhietDo, sizeof(dataA.nhietDo), "%.2f", t);
          snprintf(dataA.doAm,    sizeof(dataA.doAm),    "%.2f", h);
          strcpy(dataA.rssi, buffRSSI);
          strcpy(dataA.snr,  buffSNR);
          xQueueSend(dataQueue, &dataA, portMAX_DELAY);
        }
      }
      // Node B: "LUX:xx SOIL:xx"
      else if (inComing.startsWith("LUX:")) {
        float lux=0, soil=0;
        // Ví dụ hợp lệ: "LUX:1234.0 SOIL:58"
        if (sscanf(inComing.c_str(), "LUX:%f SOIL:%f", &lux, &soil) == 2) {
          SensorData dataB; memset(&dataB, 0, sizeof(SensorData));
          dataB.node = 'B';
          snprintf(dataB.lux,  sizeof(dataB.lux),  "%.2f", lux);
          snprintf(dataB.soil, sizeof(dataB.soil), "%.2f", soil);
          strcpy(dataB.rssi, buffRSSI);
          strcpy(dataB.snr,  buffSNR);
          xQueueSend(dataQueue, &dataB, portMAX_DELAY);
        }
      }
      // -------------------------------------------
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ================== Task WiFi + MQTT ==================
void TaskWifiMqtt(void *pvParameters) {
  SensorData received;

  wifiConnectBlocking();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnectBlocking();
      mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    }
    if (!mqttClient.connected()) {
      mqttEnsureConnected();
    }
    mqttClient.loop();

    // Nhận dữ liệu từ queue và publish theo node
    if (xQueueReceive(dataQueue, &received, pdMS_TO_TICKS(500)) == pdPASS) {
      if (received.node == 'A') {
        // Publish chuỗi
        mqttClient.publish("esp32/temperature", received.nhietDo);
        mqttClient.publish("esp32/humidity",    received.doAm);
        // (tuỳ chọn) publish chất lượng link
        mqttClient.publish("esp32/a_rssi", received.rssi);
        mqttClient.publish("esp32/a_snr",  received.snr);
        Serial.printf("PUB A: T=%s  H=%s  (RSSI=%s, SNR=%s)\n",
                      received.nhietDo, received.doAm, received.rssi, received.snr);
      } else if (received.node == 'B') {
        mqttClient.publish("esp32/lux",  received.lux);
        mqttClient.publish("esp32/soil", received.soil);
        mqttClient.publish("esp32/b_rssi", received.rssi);
        mqttClient.publish("esp32/b_snr",  received.snr);
        Serial.printf("PUB B: LUX=%s  SOIL=%s  (RSSI=%s, SNR=%s)\n",
                      received.lux, received.soil, received.rssi, received.snr);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  dataQueue = xQueueCreate(5, sizeof(SensorData));

  xTaskCreatePinnedToCore(TaskLoRa,     "TaskLoRa",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskWifiMqtt, "TaskWifiMqtt", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // không dùng
}
