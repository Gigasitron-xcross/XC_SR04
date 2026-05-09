/*
 * BasicSingleSensor
 *
 * XC_SR04 example for one HC-SR04 / SR04 ultrasonic sensor.
 *
 * NUCLEO-G474RE wiring example:
 *   HC-SR04 VCC   -> 5V
 *   HC-SR04 GND   -> GND
 *   HC-SR04 TRIG  -> D8  / PA9
 *   HC-SR04 ECHO  -> voltage divider -> D7 / PA8
 *
 * Important:
 *   HC-SR04 ECHO is normally 5V. STM32 GPIO is 3.3V.
 *   Use a voltage divider or level shifter.
 */

#include <XC_SR04.h>

XC_SR04 sensor1;

volatile bool resultReady = false;
volatile bool resultError = false;

float latestDistanceCm = 0.0f;
XC_SR04_Status latestStatus = XC_SR04_ERROR;

void sr04Callback(const XC_SR04_Result &result) {
  /*
   * This callback may run from interrupt context.
   * Do not use Serial.print(), delay(), Wire, SPI, or blocking code here.
   */
  latestStatus = result.status;

  if (result.status == XC_SR04_OK) {
    latestDistanceCm = result.distanceCm;
    resultReady = true;
  } else {
    resultError = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  XC_SR04_Status status = sensor1.begin(D8, D7, TIM2);

  if (status != XC_SR04_OK) {
    Serial.print("SR04 begin failed: ");
    Serial.println(XC_SR04::statusToString(status));
    while (1) {
    }
  }

  sensor1.setCallback(sr04Callback);

  Serial.println("XC_SR04 BasicSingleSensor test");
}

void loop() {
  static uint32_t lastTriggerMs = 0;

  if (!sensor1.isBusy() && millis() - lastTriggerMs >= 100) {
    lastTriggerMs = millis();

    XC_SR04_Status status = sensor1.trigger();
    if (status != XC_SR04_OK) {
      Serial.print("Trigger failed: ");
      Serial.println(XC_SR04::statusToString(status));
    }
  }

  if (resultReady) {
    resultReady = false;

    Serial.print("Distance: ");
    Serial.print(latestDistanceCm);
    Serial.println(" cm");
  }

  if (resultError) {
    resultError = false;

    Serial.print("SR04 error: ");
    Serial.println(XC_SR04::statusToString(latestStatus));
  }

  /*
   * Other tasks can run here.
   */
}
