/*
 ESP32 BLE Trilateration - Complete Working Implementation (MINIMAL CHANGES)
 - Step-by-step guided calibration for all 3 beacons
 - Live trilateration with Kalman filtering
 - Serial monitor output for RSSI values
 - Responsive web interface with real-time map
*/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <ArduinoJson.h>
#include <Preferences.h>
Preferences prefs;


// --- UPDATE YOUR WI-FI CREDENTIALS ---
const char* ssid = "YOUR_SSID_HERE";
const char* password = "YOUR_PASSWORD_HERE";


// --- PROJECT CONFIGURATION ---
const char* beacon_names[] = {"Beacon1", "Beacon2", "Beacon3"};
const int NUM_BEACONS = 3;
#define RSSI_SAMPLE_COUNT 20
#define SCAN_INTERVAL_MS 100
#define MEASUREMENT_TIMEOUT_MS 20000


// --- WEB SERVER & WEBSOCKET ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


// --- 1D KALMAN FILTER CLASS ---
class KalmanFilter1D {
 private:
  float q; // Process variance
  float r; // Measurement variance 
  float x; // Value
  float p; // Estimation error covariance
  float k; // Kalman gain
  
 public:
  KalmanFilter1D(float process_variance = 0.5, float measurement_variance = 2.0, float initial_value = -70.0) {
   q = process_variance;
   r = measurement_variance;
   x = initial_value;
   p = 1.0;
  }
  
  float update(float measurement) {
   // Prediction update
   p = p + q;
   
   // Measurement update
   k = p / (p + r);
   x = x + k * (measurement - x);
   p = (1 - k) * p;
   
   return x;
  }
  
  float getValue() { return x; }
};


// --- ENHANCED STATE MACHINE ---
enum SystemState { 
 IDLE, 
 CALIBRATING_STEP_1_START,  // Ask user to position for Beacon1
 CALIBRATING_STEP_1_MEASURING, // Measuring Beacon1 at 1m
 CALIBRATING_STEP_2_START,  // Ask user to position for Beacon2
 CALIBRATING_STEP_2_MEASURING, // Measuring Beacon2 at 3m
 CALIBRATING_STEP_3_START,  // Ask user to position for Beacon3
 CALIBRATING_STEP_3_MEASURING, // Measuring Beacon3 at 3m
 CALIBRATION_COMPLETE,
 LOCALIZING
};


SystemState currentState = IDLE;
int current_beacon_step = 0;


// --- BLE & MEASUREMENT VARIABLES ---
BLEScan* pBLEScan;
KalmanFilter1D* kalman_filters[NUM_BEACONS];


// Calibration data
float measured_power_1m = -60.0;  // RSSI at 1 meter
float env_factors[NUM_BEACONS];   // Environmental factor for each beacon
float beacon_distance = 3.0;    // Distance between beacons
float clamp_power(float value) {
  if (value < -70.0) return -70.0;
  if (value > -45.0) return -45.0;
  return value;
}

float clamp_env(float value) {
  if (value < 1.2) return 1.2;
  if (value > 2.8) return 2.8;
  return value;
}


// Measurement state
volatile int rssi_reading_count = 0;
volatile long rssi_reading_total = 0;
unsigned long measurement_start_time = 0;
unsigned long last_scan_time = 0;
int target_beacon_idx = 0;


// Live positioning
float current_position[2] = {1.5, 1.0};
unsigned long last_position_update = 0;


// --- COMPLETE HTML PAGE (ONLY UNICODE CHARS REMOVED AND ZOOM ADDED) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
 <title>ESP32 BLE Localization</title>
 <meta name="viewport" content="width=device-width, initial-scale=1">
 <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
 <script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom"></script>
 <style>
  body { font-family: Arial, sans-serif; margin: 0; padding: 10px; background-color: #f0f2f5; }
  .container { max-width: 900px; margin: auto; background: white; padding: 20px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
  h1 { color: #2c3e50; text-align: center; margin-bottom: 30px; }
  
  #controls { 
   display: flex; justify-content: center; align-items: center; flex-wrap: wrap; 
   margin-bottom: 20px; padding: 20px; border: 2px solid #3498db; border-radius: 10px; 
   background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
  }
  #controls > * { margin: 8px 12px; }
  
  .status-box {
   background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 8px;
   font-weight: bold; text-align: center; margin-bottom: 20px;
  }
  
  label { font-weight: bold; color: #2c3e50; } 
  input { width: 60px; padding: 8px; border-radius: 6px; border: 2px solid #bdc3c7; }
  
  #log { 
   height: 200px; width: 100%; border: 2px solid #34495e; border-radius: 8px; 
   padding: 15px; box-sizing: border-box; background-color: #1a1a1a; color: #00ff41; 
   font-family: 'Courier New', monospace; font-size: 14px; white-space: pre-wrap; 
   overflow-y: scroll; margin-bottom: 20px; line-height: 1.4;
  }
  
  button { 
   padding: 12px 18px; border-radius: 8px; border: none; color: white; 
   cursor: pointer; font-size: 16px; font-weight: bold; transition: all 0.3s;
   box-shadow: 0 2px 4px rgba(0,0,0,0.2);
  }
  button:hover { transform: translateY(-1px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
  
  .btn-blue { background: linear-gradient(135deg, #3498db, #2980b9); }
  .btn-green { background: linear-gradient(135deg, #2ecc71, #27ae60); }
  .btn-red { background: linear-gradient(135deg, #e74c3c, #c0392b); }
  .btn-orange { background: linear-gradient(135deg, #f39c12, #e67e22); }
  
  button:disabled { 
   background: #95a5a6 !important; cursor: not-allowed; 
   transform: none !important; box-shadow: none !important; 
  }
  
  .hidden { display: none; }
  .chart-container { margin-top: 20px; }
  
  .progress-bar {
   width: 100%; height: 20px; background-color: #ecf0f1; border-radius: 10px;
   margin: 10px 0; overflow: hidden;
  }
  .progress-fill {
   height: 100%; background: linear-gradient(90deg, #3498db, #2ecc71);
   width: 0%; transition: width 0.5s ease;
  }
 </style>
</head>
<body>
 <div class="container">
  <h1>Live BLE Indoor Localization System</h1>
  
  <div class="status-box" id="statusBox">
   Ready for Calibration - Press "Start Calibration" to begin
  </div>
  
  <div id="controls">
   <button id="calBtn" class="btn-blue" onclick="startCalibration()">Start Calibration</button>
   <button id="nextBtn" class="btn-orange hidden" onclick="nextStep()">Next Step</button>
   <button id="locBtn" class="btn-green" onclick="toggleLocalization()" disabled>Start Localization</button>
   <button id="resetBtn" class="btn-red" onclick="resetSystem()">Reset (Forced)</button>


   <div>
    <label>Beacon Distance:</label>
    <input type="number" id="beaconDist" value="3.0" step="0.1" onchange="updateBeaconPositions()">m
   </div>
  </div>
  
  <div class="progress-bar">
   <div class="progress-fill" id="progressFill"></div>
  </div>
  
  <div id="log">System Ready!
Waiting for calibration to start...
Make sure all 3 beacons are powered on!
</div>
  
  <div class="chart-container">
   <canvas id="localizationChart"></canvas>
  </div>
 </div>


<script>
 let gateway = `ws://${window.location.hostname}/ws`;
 let websocket;
 let chart;
 let isLocalizing = false;
 let calibrationStep = 0;
 
 function resetSystem() {
  log("Sending forced reset command...");
  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify({command: 'reset_calibration'}));
    updateStatus("Reset command sent to ESP32.");
  } else {
    log("WebSocket not connected! Reload your browser.");
    updateStatus("WebSocket not connected! Reload page.");
  }
}






 window.addEventListener('load', () => { 
  initWebSocket(); 
  initChart(); 
  updateProgress(0);
 });


 function initWebSocket() {
  console.log('Connecting to WebSocket:', gateway);
  websocket = new WebSocket(gateway);
  websocket.onopen = (e) => {
   log("Connected to ESP32!");
   updateStatus("Connected - Ready for calibration");
  };
  websocket.onclose = (e) => { 
   log("Connection lost. Retrying..."); 
   updateStatus("Connection lost - Retrying...");
   setTimeout(initWebSocket, 2000); 
  };
  websocket.onmessage = onMessage;
  websocket.onerror = (e) => {
   log("WebSocket Error: " + e);
  };
 }


 function log(message, dynamic = false) {
  const logBox = document.getElementById('log');
  const timestamp = new Date().toLocaleTimeString();
  
  if (dynamic) {
   const lines = logBox.innerHTML.split('\n');
   lines[lines.length - 2] = `[${timestamp}] ${message}`;
   logBox.innerHTML = lines.join('\n');
  } else {
   logBox.innerHTML += `[${timestamp}] ${message}\n`;
  }
  logBox.scrollTop = logBox.scrollHeight;
 }


 function updateStatus(message) {
  document.getElementById('statusBox').textContent = message;
 }


 function updateProgress(percent) {
  document.getElementById('progressFill').style.width = percent + '%';
 }


 function onMessage(event) {
  try {
   const data = JSON.parse(event.data);
   console.log('Received:', data);
   
   if (data.type === 'log') { 
    log(data.message); 
   }
   else if (data.type === 'log_dynamic') { 
    log(data.message, true); 
   }
   else if (data.type === 'status') {
    updateStatus(data.message);
   }
   else if (data.type === 'progress') {
    updateProgress(data.percent);
   }
   else if (data.type === 'ui_update') { 
    updateUI(data.ui); 
   }
   else if (data.type === 'position_update') { 
    updateChart(data); 
   }
   else if (data.type === 'calibration_step') {
    calibrationStep = data.step;
    updateProgress((data.step / 4) * 100);
   }
  } catch (e) {
   console.error('JSON parse error:', e);
  }
 }


 function updateUI(ui) {
  document.getElementById('calBtn').style.display = ui.calBtn ? 'inline-block' : 'none';
  document.getElementById('nextBtn').style.display = ui.nextBtn ? 'inline-block' : 'none';
  document.getElementById('locBtn').disabled = !ui.locBtnEnabled;
  
  if (ui.nextBtn) {
   document.getElementById('nextBtn').style.display = 'inline-block';
  } else {
   document.getElementById('nextBtn').style.display = 'none';
  }
 }


 function sendMessage(obj) { 
  if (websocket.readyState === WebSocket.OPEN) {
   websocket.send(JSON.stringify(obj)); 
   console.log('Sent:', obj);
  } else {
   log("WebSocket not connected!");
  }
 }


 function startCalibration() {
  log("Attempting to start calibration...");
  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify({command: 'start_calibration'}));
    log("Sent start_calibration to ESP32.");
  } else {
    log("WebSocket not connected! Please refresh the page and try again.");
    updateStatus("WebSocket not connected! Reload page.");
  }
}



 function nextStep() {
  log("Proceeding to next step...");
  if (websocket.readyState === WebSocket.OPEN) {
    websocket.send(JSON.stringify({command: 'next_step'}));
    log("Sent next_step to ESP32.");
  } else {
    log("WebSocket not connected! Please refresh and try again.");
    updateStatus("WebSocket not connected! Reload page.");
  }
}



 function toggleLocalization() {
  isLocalizing = !isLocalizing;
  const btn = document.getElementById('locBtn');
  
  if (isLocalizing) {
   btn.textContent = 'Stop Localization';
   btn.className = 'btn-red';
   log("Starting live localization...");
  } else {
   btn.textContent = 'Start Localization';
   btn.className = 'btn-green';
   log("Localization stopped.");
  }
  
  sendMessage({command: 'toggle_localization'});
 }


 function initChart() {
  const ctx = document.getElementById('localizationChart').getContext('2d');
  chart = new Chart(ctx, {
   type: 'scatter',
   data: { 
    datasets: [
     { 
      label: 'Beacons', 
      data: [], 
      backgroundColor: '#3498db', 
      pointRadius: 12,
      pointStyle: 'triangle'
     },
     { 
      label: 'Your Position', 
      data: [], 
      backgroundColor: '#e74c3c', 
      pointStyle: 'star', 
      pointRadius: 15 
     },
     { 
      label: 'Range B1', 
      data: [], 
      borderColor: 'rgba(52, 152, 219, 0.3)', 
      backgroundColor: 'rgba(52, 152, 219, 0.1)',
      borderWidth: 2, 
      fill: false,
      showLine: true,
      pointRadius: 0
     },
     { 
      label: 'Range B2', 
      data: [], 
      borderColor: 'rgba(46, 204, 113, 0.3)', 
      backgroundColor: 'rgba(46, 204, 113, 0.1)',
      borderWidth: 2, 
      fill: false,
      showLine: true,
      pointRadius: 0
     },
     { 
      label: 'Range B3', 
      data: [], 
      borderColor: 'rgba(155, 89, 182, 0.3)', 
      backgroundColor: 'rgba(155, 89, 182, 0.1)',
      borderWidth: 2, 
      fill: false,
      showLine: true,
      pointRadius: 0
     }
    ]
   },
   options: { 
    responsive: true,
    aspectRatio: 1.2,
    plugins: {
     zoom: {
      pan: {
       enabled: true,
       mode: 'xy'
      },
      zoom: {
       wheel: {
        enabled: true,
       },
       pinch: {
        enabled: true
       },
       mode: 'xy',
      }
     },
     legend: { 
      labels: { font: { size: 12 } },
      position: 'top'
     },
     tooltip: {
      callbacks: {
       label: function(context) {
        return `${context.dataset.label}: (${context.parsed.x.toFixed(2)}, ${context.parsed.y.toFixed(2)})`;
       }
      }
     }
    },
    scales: { 
     x: { 
      title: { display: true, text: 'X Position (meters)' },
      grid: { color: '#ecf0f1' }
     }, 
     y: { 
      title: { display: true, text: 'Y Position (meters)' },
      grid: { color: '#ecf0f1' }
     }
    }
   }
  });
  updateBeaconPositions();
 }


 function updateBeaconPositions() {
  const dist = parseFloat(document.getElementById('beaconDist').value);
  const height = (Math.sqrt(3) / 2) * dist;
  
  const beaconCoords = [ 
   {x: 0, y: 0, label: "Beacon1"}, 
   {x: dist, y: 0, label: "Beacon2"}, 
   {x: dist / 2, y: height, label: "Beacon3"} 
  ];
  
  chart.data.datasets[0].data = beaconCoords;
  
  const margin = 1;
  chart.options.scales.x.min = -margin;
  chart.options.scales.x.max = dist + margin;
  chart.options.scales.y.min = -margin;
  chart.options.scales.y.max = height + margin;
  
  chart.update();
 }


 function updateChart(data) {
  // Update position
  chart.data.datasets[1].data = [{ 
   x: data.position.x, 
   y: data.position.y 
  }];
  
  // Update range circles
  const beaconCoords = chart.data.datasets[0].data;
  const colors = ['#3498db', '#2ecc71', '#9b59b6'];
  
  for (let i = 0; i < 3 && i < data.distances.length; i++) {
   const circleData = [];
   const radius = data.distances[i];
   
   for (let angle = 0; angle <= 2 * Math.PI; angle += 0.2) {
    circleData.push({
     x: beaconCoords[i].x + radius * Math.cos(angle),
     y: beaconCoords[i].y + radius * Math.sin(angle)
    });
   }
   chart.data.datasets[i + 2].data = circleData;
  }
  
  chart.update('none');
 }
</script>
</body>
</html>
)rawliteral";


// --- BLE CALLBACK ---
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
   String deviceName = advertisedDevice.getName().c_str();
   
   for (int i = 0; i < NUM_BEACONS; i++) {
    if (deviceName == beacon_names[i]) {
     int rssi = advertisedDevice.getRSSI();
     
     // Print to Serial Monitor (removed weird chars)
     Serial.printf("%s: RSSI = %d dBm", beacon_names[i], rssi);
     
     // Handle calibration measurements
     if ((currentState == CALIBRATING_STEP_1_MEASURING && i == 0) || 
       (currentState == CALIBRATING_STEP_2_MEASURING && i == 1) ||
       (currentState == CALIBRATING_STEP_3_MEASURING && i == 2)) {
      if (i == target_beacon_idx) {
       rssi_reading_total += rssi;
       rssi_reading_count++;
       Serial.printf(" (Sample %d/%d)", rssi_reading_count, RSSI_SAMPLE_COUNT);
      }
     }
     // Handle live localization
     else if (currentState == LOCALIZING) {
      float filtered_rssi = kalman_filters[i]->update((float)rssi);
      Serial.printf(" -> Filtered: %.1f dBm", filtered_rssi);
     }
     
     Serial.println();
     break;
    }
   }
  }
};


// --- UTILITY FUNCTIONS ---
float rssi_to_distance(float rssi, float measured_power, float env_factor) {
  if (env_factor == 0) env_factor = 2.0; // Default path loss exponent
  return pow(10.0, (measured_power - rssi) / (10.0 * env_factor));
}


void trilaterate(float x1, float y1, float r1, 
        float x2, float y2, float r2, 
        float x3, float y3, float r3, 
        float* result_x, float* result_y) {
  
  float A = 2 * (x2 - x1);
  float B = 2 * (y2 - y1);
  float C = pow(r1, 2) - pow(r2, 2) - pow(x1, 2) + pow(x2, 2) - pow(y1, 2) + pow(y2, 2);
  float D = 2 * (x3 - x2);
  float E = 2 * (y3 - y2);
  float F = pow(r2, 2) - pow(r3, 2) - pow(x2, 2) + pow(x3, 2) - pow(y2, 2) + pow(y3, 2);
  
  float denominator = A * E - B * D;
  
  if (abs(denominator) < 0.0001) {
    // Lines are parallel, use last known position
    *result_x = current_position[0];
    *result_y = current_position[1];
    return;
  }
  
  *result_x = (C * E - F * B) / denominator;
  *result_y = (A * F - D * C) / denominator;
  
  // Sanity check - limit to reasonable bounds
  *result_x = constrain(*result_x, -2.0, beacon_distance + 2.0);
  *result_y = constrain(*result_y, -2.0, beacon_distance + 2.0);
  
  current_position[0] = *result_x;
  current_position[1] = *result_y;
}


void ws_send_json(JsonDocument& doc) {
  String json_string;
  serializeJson(doc, json_string);
  ws.textAll(json_string);
  Serial.println("Sent: " + json_string);
}


void ws_log(String msg, bool dynamic = false) {
  StaticJsonDocument<512> doc;
  doc["type"] = dynamic ? "log_dynamic" : "log";
  doc["message"] = msg;
  ws_send_json(doc);
}


void ws_status(String msg) {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["message"] = msg;
  ws_send_json(doc);
}


void ws_progress(int percent) {
  StaticJsonDocument<256> doc;
  doc["type"] = "progress";
  doc["percent"] = percent;
  ws_send_json(doc);
}


void update_ui() {
  StaticJsonDocument<256> doc;
  doc["type"] = "ui_update";
  JsonObject ui = doc.createNestedObject("ui");
  
  ui["calBtn"] = (currentState == IDLE || currentState == CALIBRATION_COMPLETE);
  ui["nextBtn"] = (currentState == CALIBRATING_STEP_1_START || 
          currentState == CALIBRATING_STEP_2_START || 
          currentState == CALIBRATING_STEP_3_START);
  ui["locBtnEnabled"] = (currentState == IDLE || currentState == LOCALIZING || currentState == CALIBRATION_COMPLETE);
  
  ws_send_json(doc);
}


// --- NON-BLOCKING MEASUREMENT ---
void start_measurement(int beacon_idx) {
  target_beacon_idx = beacon_idx;
  rssi_reading_count = 0;
  rssi_reading_total = 0;
  measurement_start_time = millis();
  last_scan_time = 0;
  
  Serial.printf("Starting measurement for %s...\n", beacon_names[beacon_idx]);
  ws_log("Measuring " + String(beacon_names[beacon_idx]) + "...");
}


bool update_measurement() {
  unsigned long current_time = millis();
  // FIRST: Check if measurement is complete
  if (rssi_reading_count >= RSSI_SAMPLE_COUNT) {
    ws_log("", true);  // Clear dynamic log
    return false;    // Measurement complete
  }
  // Check for timeout
  if (current_time - measurement_start_time > MEASUREMENT_TIMEOUT_MS) {
    ws_log("Measurement timeout!");
    return false;
  }
  
  // Perform periodic scans
  if (current_time - last_scan_time >= SCAN_INTERVAL_MS) {
    last_scan_time = current_time;
    pBLEScan->start(1, false);
    
    // Update progress
    String progress = "Reading " + String(rssi_reading_count) + "/" + String(RSSI_SAMPLE_COUNT) + "...";
    ws_log(progress, true);
  }
  
  // Check if measurement is complete
  if (rssi_reading_count >= RSSI_SAMPLE_COUNT) {
    ws_log("", true); // Clear dynamic message
    return false; // Measurement complete
  }
  
  return true; // Continue measuring
}


float get_measurement_result() {
  if (rssi_reading_count < 5) {
    Serial.println("Insufficient readings!");
    return -999.0;
  }
  
  float average = (float)rssi_reading_total / rssi_reading_count;
  Serial.printf("Average RSSI: %.2f dBm from %d samples\n", average, rssi_reading_count);
  return average;
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;


    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, (char*)data);


    if (error) {
      Serial.println("JSON parse error: " + String(error.c_str()));
      return;
    }


    const char* cmd = doc["command"];
    Serial.println("Command: " + String(cmd) + " | Current State: " + String(currentState));


    if (strcmp(cmd, "start_calibration") == 0) {
      if (currentState == IDLE || currentState == CALIBRATION_COMPLETE) {
        Serial.println("Starting calibration from beginning");
        currentState = CALIBRATING_STEP_1_START;
        current_beacon_step = 1;
        ws_progress(0);
        ws_log("STEP 1/3: Calibration Setup");
        ws_log("Place the ESP32 exactly 1 meter away from Beacon1");
        ws_log("Ensure clear line of sight between ESP32 and Beacon1");
        ws_log("Press 'Next Step' when positioned correctly");
        ws_status("Step 1: Position ESP32 1m from Beacon1");
        update_ui();
      }
    }
    else if (strcmp(cmd, "next_step") == 0) {
      Serial.println("Next step pressed. Current state: " + String(currentState));


      if (currentState == CALIBRATING_STEP_1_START) {
        Serial.println("Transitioning to STEP_1_MEASURING");
        currentState = CALIBRATING_STEP_1_MEASURING;
        start_measurement(0); // Beacon1 at 1m
        ws_progress(25);
        update_ui();
      }
      else if (currentState == CALIBRATING_STEP_2_START) {
        Serial.println("Transitioning to STEP_2_MEASURING");
        currentState = CALIBRATING_STEP_2_MEASURING;
        start_measurement(1); // Beacon2 at 3m
        ws_progress(50);
        update_ui();
      }
      else if (currentState == CALIBRATING_STEP_3_START) {
        Serial.println("Transitioning to STEP_3_MEASURING");
        currentState = CALIBRATING_STEP_3_MEASURING;
        start_measurement(2); // Beacon3 at 3m
        ws_progress(75);
        update_ui();
      }
      else {
        Serial.println("Next step pressed but not in valid state: " + String(currentState));
        ws_log("Invalid state for next step. Please restart calibration.");
      }
    }
    else if (strcmp(cmd, "toggle_localization") == 0) {
      if (currentState == LOCALIZING) {
        currentState = IDLE;
        ws_status("Localization stopped");
      } else if (currentState == CALIBRATION_COMPLETE || currentState == IDLE) {
        currentState = LOCALIZING;
        ws_status("Live localization active");
        last_position_update = millis();
      }
      update_ui();
    }
    else if (strcmp(cmd, "reset_calibration") == 0) {
      prefs.begin("bleloc", false);
      prefs.putFloat("power", -60.0);
      for(int i=0; i<NUM_BEACONS; i++) prefs.putFloat(("env" + String(i)).c_str(), 2.5);
      prefs.end();
      ws_status("Calibration data reset. System IDLE.");
      currentState = IDLE;
      update_ui();
    }
  }
}



void onEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *a, uint8_t *d, size_t l) {
  switch (t) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", c->id(), c->remoteIP().toString().c_str());
      ws_log("Welcome! Ready for calibration.");
      ws_status("Connected - Ready for calibration");
      update_ui();
      break;
      
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", c->id());
      break;
      
    case WS_EVT_DATA:
      handleWebSocketMessage(a, d, l);
      break;
      
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}


// --- MAIN SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 BLE Trilateration System Starting...");
  
  // Initialize Kalman filters
  for (int i = 0; i < NUM_BEACONS; i++) {
    kalman_filters[i] = new KalmanFilter1D(0.5, 2.0, -70.0);
    Serial.printf("Kalman filter %d initialized\n", i + 1);
  }
  
  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  Serial.println("BLE initialized");
  
  // Connect to WiFi
  Serial.println("Connecting to WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  Serial.println("Access the web interface at: http://" + WiFi.localIP().toString());
  
  // Setup web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Web page requested");
    request->send_P(200, "text/html", index_html);
  });
  
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  
  Serial.println("System ready! Open the web interface to start calibration.");
  Serial.println("RSSI values will be displayed here in real-time.");
  Serial.println("==================================================");
}


// --- MAIN LOOP WITH STATE MACHINE ---
void loop() {
  ws.cleanupClients();
  
  switch (currentState) {
    case IDLE:
      // Nothing to do, wait for commands
      break;
      
    case CALIBRATING_STEP_1_START:
      // Instructions are shown immediately in WebSocket handler
      // Just wait here until user presses "Next Step"
      break;
      
    case CALIBRATING_STEP_1_MEASURING:
      if (!update_measurement()) {
        float avg_rssi = get_measurement_result();
        if (avg_rssi != -999.0) {
          measured_power_1m = clamp_power(avg_rssi);
          if (measured_power_1m == -70.0 || measured_power_1m == -45.0) {
            ws_log("Warning: Reference RSSI at 1m was at limit (" + String(measured_power_1m) + " dBm), please recalibrate.");
          }

          ws_log("Step 1 Complete! Measured power at 1m: " + String(measured_power_1m, 2) + " dBm");
          Serial.println("Reference power (1m): " + String(measured_power_1m, 2) + " dBm");
          
          // Automatically move to Step 2
          currentState = CALIBRATING_STEP_2_START;
          current_beacon_step = 2;
          ws_progress(33);
          
          // Show Step 2 instructions immediately
          ws_log("STEP 2/3: Environmental Factor - Beacon2");
          ws_log("Place ESP32 exactly " + String(beacon_distance, 1) + " meters from Beacon2");
          ws_log("Maintain clear line of sight");
          ws_log("Press 'Next Step' when positioned correctly");
          ws_status("Step 2: Position ESP32 " + String(beacon_distance, 1) + "m from Beacon2");
        } else {
          ws_log("Step 1 failed! Please restart calibration.");
          currentState = IDLE;
        }
        update_ui();
      }
      break;
      
    case CALIBRATING_STEP_2_START:
      // Wait for user to press "Next Step"
      break;
      
    case CALIBRATING_STEP_2_MEASURING:
      if (!update_measurement()) {
        float avg_rssi = get_measurement_result();
        if (avg_rssi != -999.0) {
          float calibration_distance = 1.0;
          float computed_env = (measured_power_1m - avg_rssi) / (10.0 * log10(calibration_distance));
          env_factors[1] = clamp_env(computed_env);
          if (env_factors[1] == 1.2 || env_factors[1] == 2.8) {
            ws_log("Warning: Env factor for Beacon 2 at limit (" + String(env_factors[1],2) + "), please recalibrate.");
          }

          ws_log("Step 2 Complete! Environmental factor for Beacon2: " + String(env_factors[1], 2));
          Serial.println("Env factor Beacon2: " + String(env_factors[1], 2));
          
          // Automatically move to Step 3
          currentState = CALIBRATING_STEP_3_START;
          current_beacon_step = 3;
          ws_progress(66);
          
          // Show Step 3 instructions immediately
          ws_log("STEP 3/3: Environmental Factor - Beacon3");
          ws_log("Place ESP32 exactly " + String(beacon_distance, 1) + " meters from Beacon3");
          ws_log("Maintain clear line of sight");
          ws_log("Press 'Next Step' when positioned correctly");
          ws_status("Step 3: Position ESP32 " + String(beacon_distance, 1) + "m from Beacon3");
        } else {
          ws_log("Step 2 failed! Please restart calibration.");
          currentState = IDLE;
        }
        update_ui();
      }
      break;
      
    case CALIBRATING_STEP_3_START:
      // Wait for user to press "Next Step"
      break;
      
    case CALIBRATING_STEP_3_MEASURING:
  // Only proceed if measurement update completes (returns false)
  if (!update_measurement()) {
    float avg_rssi = get_measurement_result();

    // Check for error value in the measurement
    if (avg_rssi != -999.0) {
      // Calculate environmental factor for Beacon 3
      env_factors[2] = (measured_power_1m - avg_rssi) / (10.0 * log10(beacon_distance));
      env_factors[2] = clamp_env(env_factors[2]); // Ensure factor stays in valid range

      // Notify if at a limit, to prompt recalibration
      if (env_factors[2] <= 1.2 || env_factors[2] >= 2.8) {
        ws_log("Warning: Env factor for Beacon 3 at limit (" + String(env_factors[2], 2) + "), please recalibrate.");
      }

      ws_log("Step 3 Complete! Environmental factor for Beacon3: " + String(env_factors[2], 2));
      Serial.println("Env factor Beacon3: " + String(env_factors[2], 2));

      // Estimate default Env. Factor for Beacon1 (as average of Beacon2 and Beacon3)
      env_factors[0] = (env_factors[1] + env_factors[2]) / 2.0;

      // Finalize calibration process
      currentState = CALIBRATION_COMPLETE;
      ws_progress(100);

      ws_log("CALIBRATION COMPLETE!");
      ws_log("Final Parameters:");
      ws_log("  Reference Power (1m): " + String(measured_power_1m, 2) + " dBm");
      ws_log("  Env Factor Beacon1: " + String(env_factors[0], 2));
      ws_log("  Env Factor Beacon2: " + String(env_factors[1], 2));
      ws_log("  Env Factor Beacon3: " + String(env_factors[2], 2));
      ws_log("Ready for live localization!");
      ws_status("Calibration Complete - Ready for localization");

      Serial.println("Calibration complete!");
      Serial.println("System parameters:");
      for (int i = 0; i < NUM_BEACONS; i++) {
        Serial.printf("  %s env factor: %.2f\n", beacon_names[i], env_factors[i]);
      }
    } else {
      // Measurement failed, cleanly reset state and prompt user to restart
      ws_log("Step 3 failed! Please restart calibration.");
      currentState = IDLE;
    }
    update_ui(); // Always update UI after step
  }
  break;

      
    case CALIBRATION_COMPLETE:
      // Wait for user to start localization
      break;
      
    case LOCALIZING:
      // Perform live localization
      static unsigned long last_localization_scan = 0;
      if (millis() - last_localization_scan >= 500) { // Update every 500ms
        last_localization_scan = millis();
        pBLEScan->start(1, false);
        
        // Calculate distances using Kalman-filtered RSSI
        float distances[NUM_BEACONS];
        bool valid_data = true;
        
        for (int i = 0; i < NUM_BEACONS; i++) {
          float filtered_rssi = kalman_filters[i]->getValue();
          float safe_env = clamp_env(env_factors[i]);
          float safe_power = clamp_power(measured_power_1m);
          distances[i] = rssi_to_distance(filtered_rssi, safe_power, safe_env);
          distances[i] = constrain(distances[i], 0.3, 10.0);
          Serial.printf("[LIVE] Beacon%d RSSI %.1f, RSSI0 %.1f, n %.2f, dist %.2f\n",
            i+1, filtered_rssi, safe_power, safe_env, distances[i]);

          if (distances[i] < 0.3 || distances[i] > 10.0) {
            ws_log("Warning: Live distance for Beacon " + String(i+1) + " out of range.");
            valid_data = false;
            break;
  }
}

        
        if (valid_data) {
          // Beacon positions (equilateral triangle)
          float b1_x = 0, b1_y = 0;
          float b2_x = beacon_distance, b2_y = 0;
          float b3_x = beacon_distance / 2, b3_y = (sqrt(3) / 2) * beacon_distance;
          
          float pos_x, pos_y;
          trilaterate(b1_x, b1_y, distances[0],
               b2_x, b2_y, distances[1],
               b3_x, b3_y, distances[2],
               &pos_x, &pos_y);
          
          // Send position update
          StaticJsonDocument<512> doc;
          doc["type"] = "position_update";
          JsonObject position = doc.createNestedObject("position");
          position["x"] = pos_x;
          position["y"] = pos_y;
          JsonArray dist_array = doc.createNestedArray("distances");
          for (int i = 0; i < NUM_BEACONS; i++) {
            dist_array.add(distances[i]);
          }
          ws_send_json(doc);
          
          // Print to serial
          Serial.printf("Position: (%.2f, %.2f) | Distances: %.2f, %.2f, %.2f\n", 
                pos_x, pos_y, distances[0], distances[1], distances[2]);
        }
      }
      break;
  }
  
  delay(10); // Small delay to prevent watchdog issues
}

