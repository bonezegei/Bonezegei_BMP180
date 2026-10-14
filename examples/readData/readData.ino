/*
  Read All Sensor value
  Author: Jofel Batutay (Bonezegei)
  Date: October 14, 2026
*/
#include <Wire.h>
#include <Bonezegei_BMP180.h>

// Initialize BMP180 with default I2C address (0x77)
Bonezegei_BMP180 bmp(0x77);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }

  Serial.println("BMP180 Sensor Initialized!");
}

void loop() {
  if (bmp.read()) {
    Serial.print("Temperature: ");
    Serial.print(bmp.getTemperature());
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(bmp.getPressure() / 100.0f); // Convert Pa to hPa
    Serial.println(" hPa");

    Serial.print("Approx Altitude: ");
    Serial.print(bmp.getAltitude());
    Serial.println(" m");

    Serial.println("------------------------------------");
  } else {
    Serial.println("Failed to read from BMP180 sensor!");
  }

  delay(2000);
}
