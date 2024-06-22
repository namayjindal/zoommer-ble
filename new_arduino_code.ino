#include "LSM6DS3.h"
#include "Wire.h"
#include <ArduinoBLE.h>

// Create an instance of the LSM6DS3 class
LSM6DS3 myIMU(I2C_MODE, 0x6A);  // I2C device address 0x6A

#define CONVERT_G_TO_MS2 9.80665f
#define FREQUENCY_HZ 25
#define INTERVAL_MS (1000 / FREQUENCY_HZ)
#define BUFFER_SIZE 100 // Adjust as needed

struct IMUData {
  unsigned int index;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
};

IMUData imuBuffer[BUFFER_SIZE];
volatile int head = 0;
volatile int tail = 0;

static unsigned long last_interval_ms = 0;
static unsigned int sensor_index = 0;
static unsigned long lastBLETransmit = 0; // Declare lastBLETransmit

BLEService uartService("9E400001-C5C3-E393-B0A9-E50E24DCCA9E"); // UART service UUID
BLECharacteristic rxCharacteristic("9E400002-C5C3-E393-B0A9-E50E24DCCA9E", BLEWrite, 20); // RX characteristic
BLECharacteristic txCharacteristic("9E400003-C5C3-E393-B0A9-E50E24DCCA9E", BLENotify, 20); // TX characteristic

void setup() {
  Serial.begin(115200);  // Initialize serial communication

  // Start BLE without waiting for Serial Monitor
  if (!BLE.begin()) {
    while (1); // Halt execution if BLE initialization fails
  }

  BLE.setDeviceName("Sense Ball");
  BLE.setLocalName("Sense Ball");
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

  // Collect IMU data
  if (millis() - last_interval_ms >= INTERVAL_MS) {
    last_interval_ms = millis();

    IMUData data;
    data.index = sensor_index++;
    data.accelX = myIMU.readFloatAccelX() * CONVERT_G_TO_MS2;
    data.accelY = myIMU.readFloatAccelY() * CONVERT_G_TO_MS2;
    data.accelZ = myIMU.readFloatAccelZ() * CONVERT_G_TO_MS2;
    data.gyroX = myIMU.readFloatGyroX();
    data.gyroY = myIMU.readFloatGyroY();
    data.gyroZ = myIMU.readFloatGyroZ();

    // Add data to buffer
    imuBuffer[head] = data;
    head = (head + 1) % BUFFER_SIZE;

    // Check for buffer overflow
    if (head == tail) {
      tail = (tail + 1) % BUFFER_SIZE; // Drop the oldest data
    }

    // Print the IMU data to the serial monitor
    Serial.print("IMU Data: ");
    Serial.print(data.index); Serial.print(",");
    Serial.print(data.accelX, 4); Serial.print(",");
    Serial.print(data.accelY, 4); Serial.print(",");
    Serial.print(data.accelZ, 4); Serial.print(",");
    Serial.print(data.gyroX, 4); Serial.print(",");
    Serial.print(data.gyroY, 4); Serial.print(",");
    Serial.print(data.gyroZ, 4);
    Serial.println();
  }

  // Transmit IMU data via BLE
  if (central && central.connected() && millis() - lastBLETransmit >= 60) { // Adjust interval as needed
    lastBLETransmit = millis();

    while (tail != head) {
      IMUData data = imuBuffer[tail];
      tail = (tail + 1) % BUFFER_SIZE;

      char imuData[100];
      snprintf(imuData, sizeof(imuData), "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               data.index, data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);

      size_t length = strlen(imuData);
      for (size_t i = 0; i < length; i += 20) {
        txCharacteristic.writeValue((const uint8_t*)(imuData + i), min(20, length - i));
        delay(15); // Ensure BLE stack can process this chunk
      }
    }
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


