#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!lox.begin(0x29)) {
    Serial.println(F("Falha ao iniciar o sensor VL53L0X! Verifique as conexões."));
  }

  Serial.println(F("Sensor VL53L0X iniciado com sucesso!"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {  // Verifica se a medição é válida
    Serial.print(F("Distância: "));
    Serial.print(measure.RangeMilliMeter);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("Medição inválida!"));
  }

  delay(1000);  // Aguarda 1 segundo antes da próxima leitura
}

// // #include <Wire.h>
// // #include <VL53L0X.h>
// VL53L0X sensor;
// VL53L0X sensor1;

// #define SCL 22
// #define SDA 21

// void setup() {
//   Serial.begin(9600);
//   Wire.begin();

//   sensor.setTimeout(500);
//   sensor1.setTimeout(500);
//   if (!sensor.init()){
//     Serial.println("Failed to detect and initialize sensor!");
//   }
//   if (!sensor1.init()){
//     Serial.println("Failed to detect and initialize sensor 1!");
//   }

//   sensor.startContinuous();
//   sensor1.startContinuous();
// }

// void loop() {
//   Serial.print("Distância: ");
//   Serial.print(sensor.readRangeContinuousMillimeters());
//   Serial.println(" mm");

//   Serial.print("Distância 1: ");
//   Serial.print(sensor1.readRangeContinuousMillimeters());
//   Serial.println(" mm");

//   if (sensor.timeoutOccurred()) { Serial.println("Erro de tempo de espera!"); }
//   if (sensor1.timeoutOccurred()) { Serial.println("Erro de tempo de espera!"); }

//   delay(100);
// }