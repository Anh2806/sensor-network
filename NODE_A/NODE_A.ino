#include "Wire.h"
#include <Adafruit_ADS1X15.h>
#include <LoRa.h>  

#define LORA_SS   15
#define LORA_RST  2     
#define LORA_DIO0 16


const int ADC_PIN_LUX  = 14;
const int ADC_PIN_SOIL = 13;

const float LUX_MIN = 1.0f;
const float LUX_MAX = 6000.0f;
const int AirValue   = 17773;   
const int WaterValue = 10450;   

float glux;
float gsoil;

float luxFromADC(int val) {
  if (val < 0) val = 0;
  else if (val > 32767) val = 32767;
  float ratio = val / 32767.0f;
  float lux = LUX_MIN + ratio * (LUX_MAX - LUX_MIN);
  if (lux < LUX_MIN) lux = LUX_MIN;
  if (lux > LUX_MAX) lux = LUX_MAX;
  return lux;
}

int soilPercentFromADC(int val) {
  long p = map(val, AirValue, WaterValue, 0, 100); 
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (int)p;
}

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello");


  Wire.begin();              
  ads.setGain(GAIN_ONE);
  if (!ads.begin()) {
    Serial.println("Fail to initialize ADS");
    while (1) { delay(10); }
  }


  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  Serial.print("LoRa init...");
  int retry = 0;
  while (!LoRa.begin(433E6)) {
    Serial.print(".");
    delay(500);
    if (++retry > 20) {       
      Serial.println("\nLoRa init fail");
      while (1) { delay(500); }
    }
  }


  LoRa.receive();   
  Serial.println("\nLoRa ready, Listening....");
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
    Serial.print("RX: ");
    Serial.println(msg);

    if (msg == "NODE-2") {
 
      int16_t adc0 = ads.readADC_SingleEnded(0); 
      int16_t adc1 = ads.readADC_SingleEnded(1); 

      glux  = luxFromADC(adc0);
      gsoil = soilPercentFromADC(adc1);

      Serial.print("glux: ");  Serial.println(glux);
      Serial.print("gsoil: "); Serial.println(gsoil);

      char out[50];
      snprintf(out, sizeof(out), "LUX:%.2f SOIL:%.2f", glux, gsoil);

      LoRa.idle();         
      LoRa.beginPacket();
      LoRa.print(out);
      LoRa.endPacket();
      Serial.print("TX: "); Serial.println(out);

      LoRa.receive();      
    }
  }


  delay(2);
}
