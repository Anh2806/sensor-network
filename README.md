# **Introduction**

In Nowadays’s busy life, garden maintenance has become a difficult task. Plants need to be
monitored and taken care of by providing adequate watering, fertilizers, etc. Also, for efficient growth, the plants need optimum conditions such as adequate light, proper temperature and moisture content in the soil as wellas air. However, a lot of people are unable to take care of their gardens due to busy and hectic schedules.Also, some of these parameters are difficult to without devices. All these factors result in detriment to the plant health and damage to the overall garden. IoT can provide a solution for this problem. With the help of network infrastructure, IoT enables remote sensing and controlling of objects.Thus, an IoT based smart gardening system can provide a convenient and easy means for plant monitoring and garden
maintenance

Here you go — the full **English README.md**, clean, professional, GitHub-ready, but still modern enough so devs reading it won’t fall asleep.
(Still based on your project files + images + report structure.)

---

# 🌱 Garden Monitoring System – IoT • LoRa • Node-RED • Cloudflare Tunnel

A long-range, low-cost IoT monitoring system for small-scale gardens and greenhouses.
The system collects environmental data from remote sensor nodes via LoRa, aggregates it at a central ESP32-S3 Gateway, then publishes it to MQTT and visualizes everything in real-time using Node-RED. Data is simultaneously logged to Google Sheets for long-term analysis.

> Summary adapted from the original project documentation 

---

## 🚀 Project Overview


This project implements a complete IoT sensing architecture using:

* 📡 **ESP32-S3 LoRa Gateway (SX1278)**
* 📍 **Node A (ESP8266):** Temperature & humidity (AHT30)
* 📍 **Node B (ESP8266):** Light + soil moisture via ADS1115 16-bit ADC
* 💬 **Communication:** LoRa → MQTT → Node-RED
* 📊 **Dashboard:** Real-time gauges & charts
* ☁️ **Remote Access:** Cloudflare Tunnel (HTTPS without port forwarding)
* 📄 **Data Logging:** Google Sheets API

Designed for small farms, home gardens, greenhouses, and IoT research environments.

---

## 🧩 System Architecture

### High-level Architecture

*(corresponds to Figure 3.1 from report)*
<img width="470" height="623" alt="image" src="https://github.com/user-attachments/assets/593e2e75-be35-4114-8eef-b09aea9403b4" />


Data flow:

Node A / Node B
→ **LoRa SX1278**
→ **ESP32-S3 Gateway**
→ **MQTT Broker**
→ **Node-RED Dashboard + Google Sheets**

---

## 🎛 Key Features

### ✔ Multi-sensor environmental monitoring

* Air temperature
* Air humidity
* Light intensity
* Soil moisture (%)

### ✔ Real-time visualization (Node-RED UI)

Gauges + time-series charts
![Uploading image.png…]()

### ✔ Gateway information panel

RSSI, SNR, uptime, CPU temp, MQTT status
![Uploading image.png…]()



### ✔ Data logging to Google Sheets

![sheets](./images/google-sheets.png)

### ✔ Remote device control

(LED, fan, pump) via MQTT dashboard switches.

### ✔ Internet access via Cloudflare Tunnel

Secure HTTPS without router port-forwarding.
![cloudflare](./images/cloudflare.png)

---

## 🛠 Technologies Used

| Component             | Description                         |
| --------------------- | ----------------------------------- |
| **ESP32-S3 Gateway**  | LoRa receiver + MQTT publisher      |
| **ESP8266 Nodes**     | Sensor acquisition nodes            |
| **SX1278 LoRa**       | Long-range sub-GHz communication    |
| **AHT30**             | Air temperature & humidity sensor   |
| **ADS1115**           | 16-bit ADC for soil & light sensors |
| **Node-RED**          | Visualization & data pipeline       |
| **Mosquitto MQTT**    | Lightweight IoT messaging           |
| **Google Sheets API** | Cloud logging                       |
| **Cloudflare Tunnel** | Public & secure access              |

---

## 📡 LoRa Configuration

* Frequency: **433 / 868 / 915 MHz** (region-dependent)
* Bandwidth (BW): **125 kHz**
* Spreading Factor (SF): **7**
* Coding Rate (CR): **4/5**
* RF Power: **17 dBm**

Tested range: **300–500 m** in suburban conditions.

---

## 📂 Repository Structure

```
/NODE_A.ino         # Node A firmware (AHT30 sensor)
/NODE_B.ino         # Node B firmware (ADS1115 + light/soil)
/GATEWAY_new.ino    # ESP32-S3 Gateway firmware
/DO_AM.ino          # Sensor processing module
/report/            # Original documentation (PDF/DOCX)
/images/            # Screenshots & diagrams
README.md           # Project description
```

---

## 📊 Node-RED Dashboard

Flows include:

### **Flow 1 – Sensor data → Gauges/Charts**

![nodered1](./images/nodered-flow1.png)

### **Flow 2 – Gateway monitoring**

RSSI, SNR, network type, IP, MQTT status

### **Flow 3 – Data pipeline**

Join → Format → Google Sheets Append

---

## 📈 Experimental Results

* **LoRa RSSI:** −38 to −41 dBm (very strong)
* **SNR:** +9 to +10 dB (clean link)
* **End-to-end latency:** < 3–5 seconds
* **Google Sheets:** Continuous logging
* **Dashboard performance:** Smooth & stable

---

## 🧠 Analysis & Limitations

### Strengths

* Low-cost hardware with reliable long-range LoRa communication
* ADS1115 significantly improves measurement resolution
* Clean data pipeline → MQTT → Node-RED → Sheets
* Simple, intuitive dashboard
* Cloud access without port-forwarding

### Limitations

* Resistive soil-moisture sensor drifts over time
* LDR light sensor has low accuracy compared to BH1750
* MQTT currently without TLS
* Timestamp duplication (gateway assigns same second for multiple batches)
* No long-term field test yet

---

## 🔮 Future Improvements

* Upgrade soil sensor → capacitive type
* Implement **TLS/mTLS** for MQTT security
* Add **deep-sleep + solar power** for long-term outdoor deployment
* Implement closed-loop irrigation control (auto-pump)
* Add OTA firmware updates
* Conduct extended LoRa range & reliability mapping
* NTP-based timestamp sync for higher accuracy

---

## 👥 Team Members

* Nguyễn Trương Tuấn Anh
* Vũ Huy Hoàng
* Tống Anh Kiệt

Supervisor: **Dr. Bùi Văn Trí**

---








