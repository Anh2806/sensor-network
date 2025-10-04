#define TINY_GSM_MODEM_SIM7600
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <HardwareSerial.h>

SPIClass SPI_LORA(1);

#define LORA_CS 13
#define LORA_RST 14
#define LORA_DIO0 9
#define LORA_MISO 10
#define LORA_MOSI 11
#define LORA_SCK 12

HardwareSerial SerialAT(1);
const uint32_t SERIAL_AT_BAUD = 115200;

const char apn[] = "v-internet";
const char gprUser[] = "";
const char gprPass[] = "";

const char* MQTT_HOST = "kiettong2004.3utilities.com";
const uint16_t MQTT_PORT = 1883;

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

struct SensorData {
  char node;
  char nhietDo[16];
  char doAm[16];
  char lux[16];
  char soil[16];
  char rssi[8];
  char snr[8];
};

volatile bool gsmReady = false;

QueueHandle_t dataQueue;

static bool mqttConnectWithClientId(const char* clientId) {
  Serial.printf("MQTT: connect to %s:%u with id=%s\n", MQTT_HOST, MQTT_PORT, clientId);
  bool ok = mqtt.connect(clientId);
  if (!ok) {
    Serial.printf("MQTT connect failed, rc=%d\n", mqtt.state());
    return false;
  }
  Serial.println("MQTT connected");
  return true;
}



static bool gsmBringupBlocking() {
  Serial.println("Starting SerialAT for modem...");
  SerialAT.begin(SERIAL_AT_BAUD, SERIAL_8N1, 4, 5);
  delay(100);

  Serial.println("TinyGSM: restarting modem...");
  if (!modem.restart()) {
    Serial.println("Modem restart failed!");
    return false;
  }

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");

  Serial.printf("Connecting GPRS (APN=%s) ...\n", apn);
  if (!modem.gprsConnect(apn, gprUser, gprPass)) {
    Serial.println("GPRS connect failed");
    return false;
  }
  Serial.println("GPRS connected");
  return true;
}

void TaskLoRa(void* pvParameters) {
  while (!gsmReady) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  SPI_LORA.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI_LORA);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  Serial.println("Khởi tạo LoRa...");
  if (!LoRa.begin(433E6)) {
    Serial.println("Lỗi khởi tạo LoRa!");
    vTaskDelete(NULL);
  }
  Serial.println("LoRa đã sẵn sàng.");

  for (;;) {
    if (!gsmReady) {
      Serial.println("[LoRa] Mất Internet/MQTT -> tạm dừng...");
      while (!gsmReady) vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("[LoRa] Internet/MQTT OK -> tiếp tục");
    }

    for (;;) {
      String outGoing = "NODE-1";
      LoRa.beginPacket();
      LoRa.print(outGoing);
      LoRa.endPacket();
      LoRa.receive();
      Serial.println("Gửi lệnh: " + outGoing);
      vTaskDelay(pdMS_TO_TICKS(1000));

      outGoing = "NODE-2";
      LoRa.beginPacket();
      LoRa.print(outGoing);
      LoRa.endPacket();
      LoRa.receive();
      Serial.println("Gửi lệnh: " + outGoing);
      vTaskDelay(pdMS_TO_TICKS(1000));

      LoRa.receive();
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        String inComing = "";
        while (LoRa.available()) {
          inComing += (char)LoRa.read();
        }
        inComing.trim();
        Serial.println("Nhận: " + inComing);

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();
        char buffRSSI[8];
        snprintf(buffRSSI, sizeof(buffRSSI), "%d", rssi);
        char buffSNR[8];
        snprintf(buffSNR, sizeof(buffSNR), "%.2f", snr);

        if (inComing.startsWith("T:")) {
          float t = 0, h = 0;
          if (sscanf(inComing.c_str(), "T:%f H:%f", &t, &h) == 2) {
            SensorData dataA;
            memset(&dataA, 0, sizeof(SensorData));
            dataA.node = 'A';
            snprintf(dataA.nhietDo, sizeof(dataA.nhietDo), "%.2f", t);
            snprintf(dataA.doAm, sizeof(dataA.doAm), "%.2f", h);
            strncpy(dataA.rssi, buffRSSI, sizeof(dataA.rssi) - 1);
            strncpy(dataA.snr, buffSNR, sizeof(dataA.snr) - 1);
            if (dataQueue) {
              xQueueSend(dataQueue, &dataA, portMAX_DELAY);
            }
          }
        } else if (inComing.startsWith("LUX:")) {
          float lux = 0, soil = 0;
          if (sscanf(inComing.c_str(), "LUX:%f SOIL:%f", &lux, &soil) == 2) {
            SensorData dataB;
            memset(&dataB, 0, sizeof(SensorData));
            dataB.node = 'B';
            snprintf(dataB.lux, sizeof(dataB.lux), "%.2f", lux);
            snprintf(dataB.soil, sizeof(dataB.soil), "%.2f", soil);
            strncpy(dataB.rssi, buffRSSI, sizeof(dataB.rssi) - 1);
            strncpy(dataB.snr, buffSNR, sizeof(dataB.snr) - 1);
            if (dataQueue) {
              xQueueSend(dataQueue, &dataB, portMAX_DELAY);
            }
          }
        }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
void TaskGsmMqtt(void* pvParameters) {
  SensorData received;

  if (!gsmBringupBlocking()) {
    Serial.println("GSM bringup failed, stopping task.");
    vTaskDelete(NULL);
    return;
  }

  mqtt.setServer(MQTT_HOST, MQTT_PORT);

  for (;;) {
    if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
      Serial.println("GSM: network/GPRS lost, try re-attach...");
      if (!gsmBringupBlocking()) {
        Serial.println("GSM reattach failed, retry in 10s...");
        vTaskDelay(pdMS_TO_TICKS(10000));
        continue;
      }
    }

    if (!mqtt.connected()) {
      mqtt.connect("esp32-wifi");
    }
    mqtt.loop();
    if (mqtt.connected() && modem.isNetworkConnected() && modem.isGprsConnected()) {
      gsmReady = true;
    }
    if (xQueueReceive(dataQueue, &received, pdMS_TO_TICKS(500)) == pdPASS) {
      if (received.node == 'A') {
        mqtt.publish("esp32/temperature", received.nhietDo);
        mqtt.publish("esp32/humidity", received.doAm);
        mqtt.publish("esp32/a_rssi", received.rssi);
        mqtt.publish("esp32/a_snr", received.snr);
        Serial.printf("PUB A: T=%s  H=%s  (RSSI=%s, SNR=%s)\n",
                      received.nhietDo, received.doAm, received.rssi, received.snr);
      } else if (received.node == 'B') {
        mqtt.publish("esp32/lux", received.lux);
        mqtt.publish("esp32/soil", received.soil);
        mqtt.publish("esp32/b_rssi", received.rssi);
        mqtt.publish("esp32/b_snr", received.snr);
        Serial.printf("PUB B: LUX=%s SOIL=%s (RSSI=%s, SNR=%s)\n",
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
  if (!dataQueue) {
    Serial.println("ERR: cannot create dataQueue");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  xTaskCreatePinnedToCore(TaskLoRa, "TaskLoRa", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskGsmMqtt, "TaskGsmMqtt", 8192, NULL, 1, NULL, 0);
}

void loop() {
}
