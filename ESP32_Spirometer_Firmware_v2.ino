#include <Arduino.h>
#include "Adafruit_MLX90393.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>

#define NEOPIXEL_PIN 47
#define NUMPIXELS 1
#define PI 3.14159265f

Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();  // Top sensor (pin1 front)
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();  // Bottom sensor (pin1 right)
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName       "sensor_MIRS"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_C1_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_C2_UUID "1f9803c5-26c3-41c4-93a9-7358baa3f1c2"
#define CHARACTERISTIC_Y_UUID "55a61982-180d-40c4-9c7c-437514c66290"
BLECharacteristic *pCharacteristic1;
BLECharacteristic *pCharacteristic2;
BLECharacteristic *pCharacteristicY;

#define BATTERY_SERVICE_UUID "0000180F-0000-1000-8000-00805F9B34FB"
#define BATTERY_LEVEL_CHAR_UUID "00002A19-0000-1000-8000-00805F9B34FB"
BLECharacteristic *pBatteryCharacteristic;

bool deviceConnected = false;

const uint8_t arrayPin[4] = {38, 37, 36, 35};
const int BATTERY_PIN = 3;
const int NUM_LEDS = sizeof(arrayPin) / sizeof(arrayPin[0]);

// Battery Parameters
const float MAX_BATTERY_VOLTAGE = 4.2;  // Fully charged Li-ion battery
const float MIN_BATTERY_VOLTAGE = 3.0;  // Minimum safe voltage
const float VOLTAGE_DIVIDER_RATIO = 2.0; // 2x 1MΩ resistors (1:1 divider)
// Timing for low battery flash
unsigned long previousMillis = 0;
const long flashInterval = 1000; // 1 second flash interval
bool flashState = false;
// Battery level update timer
unsigned long lastBatteryUpdateTime = 0;
const long batteryUpdateInterval = 5000; // Update every 5 seconds

// Magnetic field tracking
float earthX = 0, earthY = 0, earthZ = 0;
float localXOffset = 0;  // Local X-axis offset
float localZOffset = 0;
const float smoothingFactor = 1;
const int initialCalibrationSamples = 100;

// Recalibration parameters
// const int recalibrationInterval = 1000;  // Recalibrate every 1000 samples
// int sampleCount = 0;
// bool calibrating = false;
// float recalibrationSum = 0;
// int recalibrationSamples = 0;

// LED control parameters
const int maxBrightness = 200;
const float deadZone = 10.0;

struct HPF {
  float prev_x = 0, prev_y = 0;
  float alpha;

  float update(float x) {
    float y = alpha * prev_y + (1 - alpha) * (x - prev_x);
    prev_x = x;
    prev_y = y;
    return y;
  }
};

HPF hpf_x;

// Sensor orientation transformation
const float sensor2Transform[3][3] = {
  { 1,  0,  0},  // X' = X
  { 0, -1,  0},  // Y' = -Y
  { 0,  0, -1}   // Z' = -Z
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pCharacteristic1->notify();
    pCharacteristic2->notify();
    pCharacteristicY->notify();

  }
};


void transformSensor2Readings(float* x, float* y, float* z) {
  float originalX = *x;
  float originalY = *y;
  float originalZ = *z;
  
  *x = sensor2Transform[0][0] * originalX + sensor2Transform[0][1] * originalY + sensor2Transform[0][2] * originalZ;
  *y = sensor2Transform[1][0] * originalX + sensor2Transform[1][1] * originalY + sensor2Transform[1][2] * originalZ;
  *z = sensor2Transform[2][0] * originalX + sensor2Transform[2][1] * originalY + sensor2Transform[2][2] * originalZ;
}

void updateNeopixel(float localX) {
  // if (calibrating) {
  //   // Blink blue during recalibration
  //   pixels.setPixelColor(0, pixels.Color(0, 0, 100));
  //   pixels.show();
  //   return;
  // }
  
  if (abs(localX) < deadZone) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  } 
  else if (localX > 0) {
    int brightness = min(maxBrightness, (int)(localX * 5));
    pixels.setPixelColor(0, pixels.Color(0, brightness, 0));
  } 
  else {
    int brightness = min(maxBrightness, (int)(-localX * 5));
    pixels.setPixelColor(0, pixels.Color(brightness, 0, 0));
  }
  pixels.show();
}

// void recalibrateOffset() {
//   calibrating = true;
//   Serial.println("Starting offset recalibration...");
  
//   // Take new offset samples
//   float sumX = 0;
//   float sumZ = 0;
//   for (int i = 0; i < initialCalibrationSamples; i++) {
//     if (i > 0.4 * initialCalibrationSamples && i <= 0.9 * initialCalibrationSamples) {
//       float x1, y1, z1, x2, y2, z2;
//       if (sensor1.readData(&x1, &y1, &z1) && sensor2.readData(&x2, &y2, &z2)) {
//         transformSensor2Readings(&x2, &y2, &z2);
//         float localX = x1 - x2;
//         float localZ = z1 - z2;
//         sumX += localX;
//         sumZ += localZ;
//       }
//     } else {
//       float x1, y1, z1, x2, y2, z2;
//       sensor1.readData(&x1, &y1, &z1); 
//       sensor2.readData(&x2, &y2, &z2);
//     }
//     //delay(20);
//   }
  
//   localXOffset = sumX / (initialCalibrationSamples * 0.5);
//   localZOffset = sumZ / (initialCalibrationSamples * 0.5);
//   sampleCount = 0;
//   calibrating = false;
  
//   Serial.print("New local X offset: ");
//   Serial.println(localXOffset, 4);
// }

float readBatteryVoltage() {
  // Read the ADC value (12-bit resolution)
  int adcValue = analogRead(BATTERY_PIN);
  
  // Convert ADC value to voltage (0-3.3V range)
  float measuredVoltage = (adcValue * 3.3) / 4095.0;
  
  // Calculate actual battery voltage considering voltage divider
  float batteryVoltage = measuredVoltage * VOLTAGE_DIVIDER_RATIO;
  
  return batteryVoltage;
}

float calculateBatteryPercentage(float voltage) {
  // Calculate percentage based on voltage range
  float percentage = 100.0 * (voltage - MIN_BATTERY_VOLTAGE) / 
                    (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
  
  // Constrain to 0-100% range
  percentage = constrain(percentage, 0.0, 100.0);
  
  return percentage;
}

void updateBatteryDisplay(float percentage) {
  // Calculate how many LEDs to light based on percentage
  int ledsToLight = map(percentage, 0, 100, 0, NUM_LEDS);
  
  // Turn on/off LEDs accordingly
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < ledsToLight) {
      // Don't turn on the first LED if we're in low battery flash mode
      if (!(percentage < 10.0 && i == 0)) {
        digitalWrite(arrayPin[i], HIGH);
      }
    } else {
      digitalWrite(arrayPin[i], LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    delay(10);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(arrayPin[i], OUTPUT);
    digitalWrite(arrayPin[i], LOW);
  }

  pixels.begin();
  pixels.setBrightness(100);
  pixels.show();

  // float fc = 0.1;
  // float dt = 0.0625;
  // hpf_x.alpha = 1.0 / (1.0 + 1.0 / (2.0 * PI * fc * dt));

  Serial.println("Starting Magnetic Field Measurement");

  // Initialize sensors
  Wire.begin(48, 34);
  
  if (!sensor1.begin_I2C()){
    Serial.println("Sensor 1 (top) not found");
    while (1) { delay(10); }
  }
  sensor1.setGain(MLX90393_GAIN_1X);
  sensor1.setResolution(MLX90393_X, MLX90393_RES_19);
  sensor1.setResolution(MLX90393_Y, MLX90393_RES_19);
  sensor1.setResolution(MLX90393_Z, MLX90393_RES_16);
  sensor1.setOversampling(MLX90393_OSR_2);
  sensor1.setFilter(MLX90393_FILTER_5);
  digitalWrite(arrayPin[0], HIGH);
  

  Wire1.begin(2, 1);
  if (!sensor2.begin_I2C(MLX90393_DEFAULT_ADDR, &Wire1)){
    Serial.println("Sensor 2 (bottom) not found");
    while (1) { delay(10); }
  }
  sensor2.setGain(MLX90393_GAIN_1X);
  sensor2.setResolution(MLX90393_X, MLX90393_RES_19);
  sensor2.setResolution(MLX90393_Y, MLX90393_RES_19);
  sensor2.setResolution(MLX90393_Z, MLX90393_RES_16);
  sensor2.setOversampling(MLX90393_OSR_2);
  sensor2.setFilter(MLX90393_FILTER_5);
  digitalWrite(arrayPin[1], HIGH);

  // Initial calibration
  
  // Serial.println("Phase 1: Calibrating Earth's field");
  // float sumX = 0, sumY = 0, sumZ = 0;
  // for (int i = 0; i < initialCalibrationSamples; i++) {
  //   if (i > 0.4*initialCalibrationSamples && i <= 0.9*initialCalibrationSamples) {
  //     float x, y, z;
  //     if (sensor2.readData(&x, &y, &z)) {
  //       transformSensor2Readings(&x, &y, &z);
  //       sumX += x;
  //       sumY += y;
  //       sumZ += z;
  //     }
  //     //delay(20);
  //   }
  // }
  // //earthX = sumX / (initialCalibrationSamples*0.5);
  // //earthY = sumY / (initialCalibrationSamples*0.5);
  // //earthZ = sumZ / (initialCalibrationSamples*0.5);
  
  // Serial.println("Measuring initial local X offset");
  // recalibrateOffset();
  // digitalWrite(arrayPin[2], HIGH);

  for (int i = 0; i < 4; i++) {
    digitalWrite(arrayPin[i], LOW);
  }
  // Configure ADC for battery monitoring
  analogReadResolution(12); // Use 12-bit resolution for better accuracy
  analogSetAttenuation(ADC_11db); // For full range up to 3.3V


  BLEDevice::init(bleServerName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID); // Environmental Sensing
  pCharacteristic1 = pService->createCharacteristic(
    CHARACTERISTIC_C1_UUID, // Analog Output
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic1->addDescriptor(new BLE2902());
  pCharacteristic2 = pService->createCharacteristic(
    CHARACTERISTIC_C2_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic2->addDescriptor(new BLE2902());
  pCharacteristicY = pService->createCharacteristic(
    CHARACTERISTIC_Y_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristicY->addDescriptor(new BLE2902());
  pService->start();

  BLEService *pBatteryService = pServer->createService(BATTERY_SERVICE_UUID);
  pBatteryCharacteristic = pBatteryService->createCharacteristic(
    BATTERY_LEVEL_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // Set initial battery level (0%)
  uint8_t initialLevel = 0;
  pBatteryCharacteristic->setValue(&initialLevel, 1);
  pBatteryService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x1F);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for connections...");
}

void loop() {
  float x1, y1, z1, x2, y2, z2;
  // 
  if (sensor1.readData(&x1, &y1, &z1) && sensor2.readData(&x2, &y2, &z2)) {
    // transformSensor2Readings(&x2, &y2, &z2);
    
    // // Update Earth's field estimate
    // earthX = (1 - smoothingFactor) * earthX + smoothingFactor * x2;
    // earthY = (1 - smoothingFactor) * earthY + smoothingFactor * y2;
    // earthZ = (1 - smoothingFactor) * earthZ + smoothingFactor * z2;
    
    // // Calculate compensated local X value
    // float localX = x1 - earthX - localXOffset;
    // float localZ = z1 - earthZ - localZOffset;

    // float localX = -1*(x1 - x2 - localXOffset);
    // float localZ = z1 + z2 - localZOffset;

    // float filteredLocalX = hpf_x.update(localX);
    
    // Update LED
    // x1 = 0.0;
    // z1 = 0.0;
    // y1 = 0.0;
    updateNeopixel(x1);

    // Print results
    if (deviceConnected) {
      //Serial.print("Adj. Local X: ");
      Serial.print(x1, 2);
      Serial.print(" ");
      //Serial.print(localZ, 2);
      //Serial.print(" ");
      Serial.print(z1, 2);
      //Serial.print(" uT | Offset: ");
      //Serial.print(localXOffset, 2);
      //Serial.print(" | Samples: ");
      //Serial.print(sampleCount);
      Serial.println();

      String data1 = "s " + String(x1, 2) + " x1 " + String(z1, 2) + " z1";
      String data2 = "s " + String(x2, 2) + " x2 " + String(z2, 2) + " z2";
      String dataY = "s " + String(y1, 2) + " y1 " + String(y2, 2) + " y2";
      // Send the data over BLE
      pCharacteristic1->setValue(data1.c_str());
      pCharacteristic1->notify();
      pCharacteristic2->setValue(data2.c_str());
      pCharacteristic2->notify();
      pCharacteristicY->setValue(dataY.c_str());
      pCharacteristicY->notify();
    }

    // Check for recalibration
    //sampleCount++;
    //if (sampleCount >= recalibrationInterval && !recalibrating) {
      //digitalWrite(arrayPin[3], HIGH);
      //recalibrateOffset();
      //digitalWrite(arrayPin[3], LOW);
    //}
  } else {
    Serial.println("Sensor read error");
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
  }
  
  float batteryVoltage = readBatteryVoltage();
  float batteryPercentage = calculateBatteryPercentage(batteryVoltage);
  // Display battery level on LEDs
  updateBatteryDisplay(batteryPercentage);

  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryUpdateTime >= batteryUpdateInterval) {
    lastBatteryUpdateTime = currentMillis;
    
    uint8_t level = (uint8_t)round(batteryPercentage);
    pBatteryCharacteristic->setValue(&level, 1);
    
    if (deviceConnected) {
      pBatteryCharacteristic->notify();
      Serial.print("BLE Battery Level Updated: ");
      Serial.print(level);
      Serial.println("%");
    }
  }

  // Check for low battery condition
  if (batteryPercentage < 10.0) {
    // Flash the FIRST LED (0-25% LED)
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= flashInterval) {
      previousMillis = currentMillis;
      flashState = !flashState;
      digitalWrite(arrayPin[0], flashState ? HIGH : LOW);
    }
  }

  // delay(50);
}