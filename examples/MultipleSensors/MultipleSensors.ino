/*
 * MultipleSensors
 *
 * Demonstrates how to create multiple XC_SR04 sensor objects.
 *
 * Important:
 *   Do not trigger ultrasonic sensors at exactly the same time unless you know
 *   what you are doing. Crosstalk can cause wrong readings.
 *
 * This example triggers two sensors sequentially.
 */

#include <XC_SR04.h>

XC_SR04 sensorA;
XC_SR04 sensorB;

volatile bool sensorAReady = false;
volatile bool sensorBReady = false;
volatile bool sensorAError = false;
volatile bool sensorBError = false;

float sensorADistanceCm = 0.0f;
float sensorBDistanceCm = 0.0f;

void sensorACallback(const XC_SR04_Result &result) {
  if (result.status == XC_SR04_OK) {
    sensorADistanceCm = result.distanceCm;
    sensorAReady = true;
  } else {
    sensorAError = true;
  }
}

void sensorBCallback(const XC_SR04_Result &result) {
  if (result.status == XC_SR04_OK) {
    sensorBDistanceCm = result.distanceCm;
    sensorBReady = true;
  } else {
    sensorBError = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  /*
   * Example pins only. Change according to your wiring.
   * Each sensor uses a different timer in this implementation.
   */
  XC_SR04_Status s1 = sensorA.begin(D8, D7, TIM2);
  XC_SR04_Status s2 = sensorB.begin(D10, D9, TIM3);

  if (s1 != XC_SR04_OK) {
    Serial.print("sensorA begin failed: ");
    Serial.println(XC_SR04::statusToString(s1));
    while (1) {}
  }

  if (s2 != XC_SR04_OK) {
    Serial.print("sensorB begin failed: ");
    Serial.println(XC_SR04::statusToString(s2));
    while (1) {}
  }

  sensorA.setCallback(sensorACallback);
  sensorB.setCallback(sensorBCallback);

  Serial.println("XC_SR04 MultipleSensors test");
}

void loop() {
  static uint32_t lastMs = 0;
  static uint8_t phase = 0;

  /*
   * Simple sequential triggering:
   *   phase 0 -> trigger sensor A
   *   phase 1 -> trigger sensor B
   */
  if (millis() - lastMs >= 100) {
    lastMs = millis();

    if (phase == 0) {
      if (!sensorA.isBusy()) {
        sensorA.trigger();
      }
      phase = 1;
    } else {
      if (!sensorB.isBusy()) {
        sensorB.trigger();
      }
      phase = 0;
    }
  }

  if (sensorAReady) {
    sensorAReady = false;
    Serial.print("Sensor A: ");
    Serial.print(sensorADistanceCm);
    Serial.println(" cm");
  }

  if (sensorBReady) {
    sensorBReady = false;
    Serial.print("Sensor B: ");
    Serial.print(sensorBDistanceCm);
    Serial.println(" cm");
  }

  if (sensorAError) {
    sensorAError = false;
    Serial.println("Sensor A error/timeout");
  }

  if (sensorBError) {
    sensorBError = false;
    Serial.println("Sensor B error/timeout");
  }
}
