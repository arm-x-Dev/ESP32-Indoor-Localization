# ESP-32 Based Indoor Localization System

## 📌 Project Overview
This project addresses the challenge of real-time indoor localization in environments where Global Navigation Satellite Systems (GNSS) like GPS are unreliable due to signal loss and multipath effects. Leveraging the ubiquitous presence of Wi-Fi and Bluetooth infrastructure, this system provides a cost-effective solution for spatial awareness and asset tracking.

The system utilizes a single **ESP32 microcontroller** that scans multiple Wi-Fi/BLE sources positioned in a defined geometric arrangement. Unlike traditional fingerprinting, this project uses **live RSSI trilateration**, offering real-time updates without the need for a pre-collected signal database.

## 🚀 Key Features
* **1D Kalman Filtering:** Executes advanced filtering on raw RSSI data per source to suppress environmental noise and transient signal fluctuations.
* **Real-Time Trilateration:** Employs embedded-friendly algorithms optimized for low-latency position calculation.
* **Interactive Web Dashboard:** A responsive UI hosted directly on the ESP32 via an asynchronous web server for live monitoring and 2D canvas mapping.
* **On-Site Calibration:** Features a guided workflow to calculate the environmental path-loss factor ($n$), adapting the system to different building materials.
* **WebSocket Connectivity:** Enables low-latency, bidirectional streaming of RSSI measurements and coordinate data.

## 🛠️ System Architecture
The hardware setup is designed for portability and rapid deployment:
* **Beacon Nodes:** Three ESP32 modules acting as stationary reference points, powered by autonomous power banks.
* **Receiver Node:** An ESP32 operating in promiscuous mode to passively capture broadcasted packets and measure RSSI.
* **Visualization:** A laptop connected via UART-over-USB to interface with the embedded web server.

## 📐 Methodology & Mathematical Models

### 1. Log-Distance Path Loss Model
The system converts signal strength (RSSI) into distance ($d$) using the following relationship:
$$d = d_0 \times 10^{\frac{RSSI_{d_0} - RSSI_d}{10n}}$$
* **$d_0$**: Reference distance (typically 1 meter).
* **$n$**: Path loss exponent, calibrated on-site to account for environmental clutter.

### 2. Kalman Filtering
A one-dimensional Kalman filter is applied to "smooth" the erratic raw RSSI signal before it is used for localization. The filter balances trust between the prediction model and new measurements to reject transient spikes.

### 3. Trilateration
The receiver's position $(x, y)$ is satisfied by solving the intersection of three circles centered at the beacon coordinates:
$$(x - x_i)^2 + (y - y_i)^2 = d_i^2$$
The system linearizes these equations into a $2 \times 2$ matrix for efficient solving on the ESP32 hardware.

## 💻 Software Requirements
* **Arduino IDE:** Used for rapid prototyping and core application logic.
* **ESP-IDF:** Utilized for lower-level hardware access and optimized Wi-Fi operations.
* **Libraries:** `ESPAsyncWebServer`, `AsyncTCP`, `ArduinoJson`, and `BLEDevice`.

## 📊 Results & Analysis
Validation was conducted in a controlled 3-meter equilateral testbed. 
* **Stability:** The Kalman filter produced a stable position cluster, accurately tracking a moving user in real-time.
* **Responsiveness:** The system demonstrated minimal lag between physical movement and dashboard updates.

---
*Developed as a Major Project for the Department of Electronics and Communication Engineering, KLEIT, Hubballi.*
