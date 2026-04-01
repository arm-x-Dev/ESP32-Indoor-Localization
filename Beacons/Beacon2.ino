/*
  BLE Beacon #2
  - Continuously broadcasts with the name "Beacon2".
  - Turns on the onboard LED to show it's active.
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>

// --- Beacon 2 Unique Name ---
#define BEACON_NAME "Beacon2"
#define ONBOARD_LED 2 // Standard pin for the onboard LED

void setup() {
  Serial.begin(115200);
  Serial.print("Starting BLE Beacon: ");
  Serial.println(BEACON_NAME);

  // Set up the onboard LED
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH); // Turn the LED on and keep it on

  BLEDevice::init(BEACON_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(0x0540); 
  BLEDevice::startAdvertising();
  
  Serial.println("Beacon started advertising.");
}

void loop() {
  // The BLE stack handles advertising in the background.
  delay(2000); 
}
