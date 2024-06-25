#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  sensor.setAddress(29);

  if (!sensor.init()){
    Serial.println("Failed to detect and initialize sensor!");
  }

  sensor.setTimeout(0);
  sensor.startContinuous();
}

void loop() {
  Serial.print("Distância: ");
  Serial.print(sensor.readRangeContinuousMillimeters());
  Serial.println(" mm");

  if (sensor.timeoutOccurred()) { Serial.println("Erro de tempo de espera!"); }

  delay(100);
}