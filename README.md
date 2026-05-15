# 📡 ESP-32 Based Indoor Localization System

![ESP32](https://img.shields.io/badge/Hardware-ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![C++](https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![License](https://img.shields.io/badge/Field-Embedded_Systems-blueviolet?style=for-the-badge)
![Status](https://img.shields.io/badge/Project-Major_Project_ECE-success?style=for-the-badge)

## 📌 Project Overview
This project addresses the challenge of real-time indoor localization in environments where **GNSS (Global Navigation Satellite Systems)** like GPS are unreliable due to signal loss and multipath effects. 

The system utilizes a single **ESP32 microcontroller** to scan multiple Wi-Fi/BLE sources in a defined geometric arrangement. Unlike traditional fingerprinting, this project uses **live RSSI trilateration**, offering real-time updates without the need for a pre-collected signal database.

---

## 🚀 System Flow and Key Features

To visualize how the code is structured, this system follows a streamlined flow from initial setup to real-time positioning:

<img width="2784" height="1536" alt="Flow" src="https://github.com/user-attachments/assets/04b0601a-5e79-4149-b731-8d419d57471a" />

### 🛠️ Technical Stack
| Component | Technology Used |
| :--- | :--- |
| **Core Microcontroller** | ESP32 (Dual-Core) |
| **Signal Processing** | 1D Kalman Filtering (RSSI Smoothing) |
| **Positioning Logic** | Real-Time Trilateration Algorithms |
| **Communication** | WebSockets (Low-latency streaming) |
| **User Interface** | Async Web Server (Hosted on-chip), HTML5 Canvas |

### ✨ Key Features:
* **Noise Suppression:** Executes advanced **Kalman Filtering** on raw RSSI data to suppress environmental noise.
* **Interactive Dashboard:** A responsive UI hosted directly on the ESP32 for live 2D mapping.
* **On-Site Calibration:** Guided workflow to calculate the **Environmental Path-Loss Factor ($n$)**.
* **Live Streaming:** Low-latency, bidirectional streaming of coordinates via WebSockets.

---

## 🏗️ System Architecture
The hardware setup is designed for portability and rapid deployment:
* **Beacon Nodes:** Three ESP32 modules acting as stationary reference points.
* **Receiver Node:** An ESP32 operating in promiscuous mode to passively capture packets.
* **Visualization:** Web-based dashboard accessible via any browser on the local network.

---

## ⚙️ Setup and Deployment

### 1. Arduino IDE Configuration
1. Open **File > Preferences**.
2. Add the Board URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Install **ESP32** via the Boards Manager.

### 2. Required Libraries
> [!IMPORTANT]
> Ensure the following libraries are installed before flashing:
* `ESPAsyncWebServer` & `AsyncTCP`
* `ArduinoJson`
* `BLEDevice` & `BLEScan`

### 3. Flashing the Code
1.  **Beacons:** Flash `Beacons/Beacon1.ino` to the three stationary nodes.
2.  **Receiver:** Update **Wi-Fi credentials** in `Receiver/Receiver.ino` and upload to the mobile node.

---

## 📐 Methodology & Mathematical Models

### 1. Log-Distance Path Loss Model
We convert signal strength (RSSI) into distance ($d$) using:
$$d = d_0 \times 10^{\frac{RSSI_{d_0} - RSSI_d}{10n}}$$
* **$n$**: Path loss exponent (calibrated on-site).

### 2. Kalman Filtering
A one-dimensional filter is applied to "smooth" erratic RSSI signals, balancing prediction models against new measurements to reject transient signal spikes.

### 3. Trilateration
The receiver's position $(x, y)$ is found by solving the intersection of three circles:
$$(x - x_i)^2 + (y - y_i)^2 = d_i^2$$

---

## 📊 Results & Analysis
* **Stability:** The Kalman filter produced a stable position cluster, effectively tracking a moving user.
* **Responsiveness:** Minimal lag achieved between physical movement and dashboard updates.

---
> **Project Credit:** > Developed as a Major Project for the Department of Electronics and Communication Engineering, **KLEIT, Hubballi**.
