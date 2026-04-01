# ESP-32 Based Indoor Localization System

## 📌 Project Overview
[cite_start]This project addresses the challenge of real-time indoor localization in environments where Global Navigation Satellite Systems (GNSS) like GPS are unreliable due to signal loss and multipath effects[cite: 5]. [cite_start]Leveraging the ubiquitous presence of Wi-Fi and Bluetooth infrastructure, this system provides a cost-effective solution for spatial awareness and asset tracking[cite: 6, 29].

[cite_start]The system utilizes a single **ESP32 microcontroller** that scans multiple Wi-Fi/BLE sources positioned in a defined geometric arrangement[cite: 9]. [cite_start]Unlike traditional fingerprinting, this project uses **live RSSI trilateration**, offering real-time updates without the need for a pre-collected signal database[cite: 10].

## 🚀 Key Features
* [cite_start]**1D Kalman Filtering:** Executes advanced filtering on raw RSSI data per source to suppress environmental noise and transient signal fluctuations[cite: 11, 227].
* [cite_start]**Real-Time Trilateration:** Employs embedded-friendly algorithms optimized for low-latency position calculation[cite: 136, 440].
* [cite_start]**Interactive Web Dashboard:** A responsive UI hosted directly on the ESP32 via an asynchronous web server for live monitoring and 2D canvas mapping[cite: 13, 14, 737].
* [cite_start]**On-Site Calibration:** Features a guided workflow to calculate the environmental path-loss factor ($n$), adapting the system to different building materials[cite: 12, 615].
* [cite_start]**WebSocket Connectivity:** Enables low-latency, bidirectional streaming of RSSI measurements and coordinate data[cite: 621, 723].

## 🛠️ System Architecture
[cite_start]The hardware setup is designed for portability and rapid deployment[cite: 704]:
* [cite_start]**Beacon Nodes:** Three ESP32 modules acting as stationary reference points, powered by autonomous power banks[cite: 425, 705].
* [cite_start]**Receiver Node:** An ESP32 operating in promiscuous mode to passively capture broadcasted packets and measure RSSI[cite: 342, 478].
* [cite_start]**Visualization:** A laptop connected via UART-over-USB to interface with the embedded web server[cite: 380, 718].

## 📐 Methodology & Mathematical Models

### 1. Log-Distance Path Loss Model
[cite_start]The system converts signal strength (RSSI) into distance ($d$) using the following relationship[cite: 493]:
$$d = d_0 \times 10^{\frac{RSSI_{d_0} - RSSI_d}{10n}}$$
* [cite_start]**$d_0$**: Reference distance (typically 1 meter)[cite: 499].
* **$n$**: Path loss exponent, calibrated on-site to account for environmental clutter[cite: 501, 515].

### 2. Kalman Filtering
A one-dimensional Kalman filter is applied to "smooth" the erratic raw RSSI signal before it is used for localization[cite: 227, 437]. The filter balances trust between the prediction model and new measurements to reject transient spikes[cite: 557, 558].

### 3. Trilateration
The receiver's position $(x, y)$ is satisfied by solving the intersection of three circles centered at the beacon coordinates[cite: 564, 565]:
$$(x - x_i)^2 + (y - y_i)^2 = d_i^2$$
The system linearizes these equations into a $2 \times 2$ matrix for efficient solving on the ESP32 hardware[cite: 588, 598].

## 💻 Software Requirements
* [cite_start]**Arduino IDE:** Used for rapid prototyping and core application logic[cite: 158, 360].
* [cite_start]**ESP-IDF:** Utilized for lower-level hardware access and optimized Wi-Fi operations[cite: 159, 364].
* **Libraries:** `ESPAsyncWebServer`, `AsyncTCP`, `ArduinoJson`, and `BLEDevice`[cite: 891].

## 📊 Results & Analysis
Validation was conducted in a controlled 3-meter equilateral testbed[cite: 804, 841]. 
* [cite_start]**Stability:** The Kalman filter produced a stable position cluster, accurately tracking a moving user in real-time[cite: 857].
* [cite_start]**Responsiveness:** The system demonstrated minimal lag between physical movement and dashboard updates[cite: 824].

---
[cite_start]*Developed as a Major Project for the Department of Electronics and Communication Engineering, KLEIT, Hubballi[cite: 16, 863].*
