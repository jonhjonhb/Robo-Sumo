#include <Arduino.h>

#define FRENTE_D 27
#define TRAS_D 26
#define PWMD 14

#define FRENTE_B 25
#define TRAS_B 33
#define PWMB 32

void aumentarVelocidade(){
  for (int i = 0; i < 256; i++){
    analogWrite(PWMD, i);
    delay(10);
  }  
}

void setup() {
  pinMode(FRENTE_D, OUTPUT);
  pinMode(TRAS_D, OUTPUT);
  pinMode(PWMD, OUTPUT);
  // pinMode(FRENTE_B, OUTPUT);
  // pinMode(TRAS_B, OUTPUT);
  // pinMode(PWMB, OUTPUT);
  // analogWrite(PWMB, 100);
}

void loop() {
  digitalWrite(FRENTE_D, HIGH);
  digitalWrite(TRAS_D, LOW);
  // digitalWrite(FRENTE_B, HIGH);
  // digitalWrite(TRAS_B, LOW);
  // aumentarVelocidade();

  delay(100);
  
  digitalWrite(FRENTE_D, LOW);
  digitalWrite(TRAS_D, HIGH);
  // digitalWrite(FRENTE_B, LOW);
  // digitalWrite(TRAS_B, HIGH);
  // aumentarVelocidade();

  delay(100);
}