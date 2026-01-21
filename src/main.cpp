#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include "Config.h"
#include "Secrets.h"

// --- OBJECTS ---
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

Adafruit_BME280 bme; 
Adafruit_MPU6050 mpu;

// --- TIMING ---
unsigned long lastMillis = 0;
const long interval = 5000; // Update every 5 seconds

// --- AWS CONNECTION ---
void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Configure MQTT with Certs
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);

  Serial.println("\nConnecting to AWS IoT");
  while (!client.connected()) {
    Serial.print(".");
    if (client.connect("ESP32_Scooter_Node")) {
      Serial.println("\nConnected!");
      client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// --- VIBRATION ANALYSIS (CHASSIS) ---
float measureVibration(int samples = 100) {
    float sum = 0;
    float readings[samples];
    
    // 1. High-Speed Burst Sampling (Z-Axis)
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        readings[i] = a.acceleration.z;
        sum += readings[i];
        delay(2); // 500Hz sampling approx
    }

    // 2. Statistical Processing
    float mean = sum / samples;
    float sqDevSum = 0.0;
    for (int i = 0; i < samples; i++) {
        sqDevSum += pow(readings[i] - mean, 2);
    }
    
    // Standard Deviation = Vibration Intensity
    return sqrt(sqDevSum / samples);
}

void setup() {
    Serial.begin(115200);

    // --- 1. CONNECT WIFI & CLOUD FIRST ---
    // We do this first so we can debug easier
    connectAWS();

    // --- 2. INITIALIZE SENSORS ---
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize Motor Sensor (BME280)
    if (!bme.begin(0x76)) { 
        if (!bme.begin(0x77)) {
            Serial.println("❌ Error: BME280 (Motor Sensor) not found!");
            // OPTIONAL: Don't halt, just print error so WiFi stays alive
        } else {
            Serial.println("✅ Motor Sensor (BME280) Ready");
        }
    } else {
        Serial.println("✅ Motor Sensor (BME280) Ready");
    }

    // Initialize Chassis Sensor (MPU6050)
    if (!mpu.begin()) {
        Serial.println("❌ Error: MPU6050 (Chassis Sensor) not found!");
        // We do NOT use while(1) here so the loop() can still run and keep WiFi alive
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        Serial.println("✅ Chassis Sensor (MPU6050) Ready");
    }
}

void loop() {
  if (!client.connected()) connectAWS();
  client.loop();

  if (millis() - lastMillis > interval) {
    lastMillis = millis();

    // --- STEP A: READ DATA ---
    // 1. Chassis Vibration (takes ~200ms)
    float vibrationIndex = measureVibration(100); 

    // 2. Motor Condition (BME280)
    float motorTemp = bme.readTemperature();
    float ambientHum = bme.readHumidity();
    float ambientPres = bme.readPressure() / 100.0F;

    // 3. Instantaneous Motion
    sensors_event_t a, g, temp_mpu;
    mpu.getEvent(&a, &g, &temp_mpu);

    // --- STEP B: BUILD JSON ---
    StaticJsonDocument<1024> doc; // Increased size for safety
    
    // Key change: specific label for Motor
    doc["motor_temp"] = motorTemp;  
    doc["humidity"] = ambientHum;
    doc["pressure"] = ambientPres;
    doc["vibration"] = vibrationIndex;

    JsonObject accel = doc.createNestedObject("accel");
    accel["x"] = a.acceleration.x;
    accel["y"] = a.acceleration.y;
    accel["z"] = a.acceleration.z;

    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = g.gyro.x;
    gyro["y"] = g.gyro.y;
    gyro["z"] = g.gyro.z;

    char jsonBuffer[1024];
    serializeJson(doc, jsonBuffer);

    // --- STEP C: UPLOAD ---
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
    
    // --- STEP D: DEBUG DISPLAY ---
    Serial.println("\n--- TELEMETRY SENT ---");
    Serial.print("Motor Temp:   "); Serial.print(motorTemp); Serial.println(" C");
    Serial.print("Vibration:    "); Serial.print(vibrationIndex); Serial.println(" m/s^2");
    Serial.print("Tilt (Acc X): "); Serial.println(a.acceleration.x);
    Serial.println("----------------------");
  }
}