#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>

// Create an instance of the LSM6DS3 class
LSM6DS3 myIMU(I2C_MODE, 0x6A);  // I2C device address 0x6A

#define CONVERT_G_TO_MS2 9.80665f
#define FREQUENCY_HZ 25
#define INTERVAL_MS (1000 / FREQUENCY_HZ)

static unsigned long last_interval_ms = 0;
static unsigned int sensor_index = 0;

BLEService uartService("8E400004-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rxCharacteristic("8E400005-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);
BLECharacteristic txCharacteristic("8E400006-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);

void setup() {
  Serial.begin(115200);  // Initialize serial communication

  // Start BLE without waiting for Serial Monitor
  if (!BLE.begin()) {
    while (1); // Halt execution if BLE initialization fails
  }

  BLE.setDeviceName("Sense Right Hand");
  BLE.setLocalName("Sense Right Hand");
  BLE.setAdvertisedService(uartService);

  // Add the RX and TX characteristics to the UART service
  uartService.addCharacteristic(rxCharacteristic);
  uartService.addCharacteristic(txCharacteristic);

  // Add the service
  BLE.addService(uartService);

  // Adjust the advertising interval to be more frequent
  BLE.setAdvertisingInterval(32); // 160 * 0.625 ms = 100 ms interval
  BLE.advertise();

  if (myIMU.begin() != 0) {
    while (1); // Halt execution if IMU initialization fails
  }
}

void loop() {
  BLEDevice central = BLE.central();

  // Always print IMU data to Serial Monitor
  if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

    float accelX = myIMU.readFloatAccelX() * CONVERT_G_TO_MS2;
    float accelY = myIMU.readFloatAccelY() * CONVERT_G_TO_MS2;
    float accelZ = myIMU.readFloatAccelZ() * CONVERT_G_TO_MS2;

    float gyroX = myIMU.readFloatGyroX();
    float gyroY = myIMU.readFloatGyroY();
    float gyroZ = myIMU.readFloatGyroZ();

    // Format IMU data with index
    char imuData[100];
    snprintf(imuData, sizeof(imuData), "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             sensor_index, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    // Send the IMU data via BLE if connected
    if (central) {
      static unsigned long lastBLETransmit = 0;
      unsigned long currentMillis = millis();

      // Check if enough time has passed since last transmission
      if (currentMillis - lastBLETransmit >= 50) {
        for (size_t i = 0; i < strlen(imuData); i += 20) {
          txCharacteristic.writeValue((const uint8_t*)(imuData + i), min(20, strlen(imuData) - i));
          // Update last transmission time
          lastBLETransmit = currentMillis;
        }
      }
    }

    // Print the IMU data to the serial monitor
    Serial.print("IMU Data: ");
    Serial.println(imuData);

    // Increment index for the next reading
    sensor_index++;
  }

  // Print connection status to Serial Monitor
  if (central && central.connected()) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
  } else if (central && !central.connected()) {
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}


