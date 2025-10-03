#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

#define LORA_SS    15
#define LORA_RST   2
#define LORA_DIO0  6

Adafruit_AHTX0 aht;

void setup() {
  Serial.begin(9600);
  delay(200);

  // I2C & AHT30
  Wire.begin();                  // nếu dùng SDA/SCL khác, đổi: Wire.begin(SDA, SCL);
  if (!aht.begin(&Wire)) {
    Serial.println("Khong tim thay AHT30! Kiem tra day/I2C.");
    // vẫn chạy để debug LoRa, nhưng sẽ không gửi được số đo
  } else {
    Serial.println("AHT30 san sang.");
  }

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init FAIL!");
    while (1) { delay(500); }
  }
  Serial.println("LoRa ready. Listening...");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String msg;
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      if (isPrintable(c)) msg += c;
    }
    msg.trim();
    Serial.print("RX: "); Serial.println(msg);

    if (msg == "NODE-1") {
      sensors_event_t hum, temp;
      if (aht.getEvent(&hum, &temp)) {
        float t = temp.temperature;
        float h = hum.relative_humidity;

        char out[48];
        snprintf(out, sizeof(out), "T:%.2f H:%.2f", t, h);

        LoRa.beginPacket();
        LoRa.print(out);
        LoRa.endPacket();

        Serial.print("TX: "); Serial.println(out);
      } else {
        Serial.println("Doc AHT30 that bai.");
      }
    }
  }

  delay(5); // nhe CPU
}
