#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- I2C PINS (ESP32 Default) ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- SENSOR SETTINGS ---
#define SEALEVELPRESSURE_HPA (1013.25)

// --- AWS TOPICS ---
// You can keep the same topics or update them for the new project
#define AWS_IOT_PUBLISH_TOPIC   "ev/scooter_01/telemetry"
#define AWS_IOT_SUBSCRIBE_TOPIC "ev/scooter_01/commands"

#endif