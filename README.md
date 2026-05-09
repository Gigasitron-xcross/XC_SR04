# XC_SR04

`XC_SR04` is an object-oriented, non-blocking HC-SR04 / SR04 ultrasonic sensor driver for **STM32duino / Arduino_Core_STM32**.

It is designed for STM32 boards such as **NUCLEO-G474RE** and avoids the common blocking `pulseIn()` approach.

## Why this library exists

Many Arduino ultrasonic sensor examples use:

```cpp
duration = pulseIn(echoPin, HIGH);
```

That works for simple demos, but `pulseIn()` is blocking. While the MCU is waiting for the ECHO pulse, the main program cannot efficiently handle other tasks such as motor PID, encoder reading, serial communication, or other sensors.

`XC_SR04` uses an interrupt-driven and timer-based approach:

```text
User calls trigger()
        ↓
Driver sends 10 us TRIG pulse
        ↓
ECHO rising edge interrupt records start time
        ↓
ECHO falling edge interrupt records end time
        ↓
Driver calculates distance
        ↓
Driver calls user callback
```

No waiting loop is required.

## Features

- Object-oriented sensor instances
- No `pulseIn()`
- Non-blocking measurement
- Interrupt-driven ECHO edge detection
- STM32 `HardwareTimer` timeout service
- Callback notification for result or error
- Timeout handling
- Status/error return codes
- Supports up to `XC_SR04_MAX_SENSORS` sensors, default 8
- Suitable for STM32duino boards such as NUCLEO-G474RE

## Hardware warning

Most HC-SR04 modules are powered from 5V and output a 5V ECHO signal.

STM32 GPIO pins are normally **3.3V only**.

Use a voltage divider or level shifter on ECHO.

Example voltage divider:

```text
HC-SR04 ECHO --- 1kΩ ---+--- STM32 ECHO pin
                         |
                        2kΩ
                         |
                        GND
```

This reduces 5V to about 3.3V.

## NUCLEO-G474RE example wiring

Example using Arduino header pins:

```text
HC-SR04 VCC   → 5V
HC-SR04 GND   → GND
HC-SR04 TRIG  → D8
HC-SR04 ECHO  → voltage divider → D7
```

For NUCLEO-G474RE:

```text
D8 = PA9
D7 = PA8
```

## Basic example

```cpp
#include <XC_SR04.h>

XC_SR04 sensor1;

volatile bool resultReady = false;
volatile bool resultError = false;

float latestDistanceCm = 0.0f;
XC_SR04_Status latestStatus = XC_SR04_ERROR;

void sr04Callback(const XC_SR04_Result &result) {
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

  Serial.println("XC_SR04 callback driver test");
}

void loop() {
  static uint32_t lastTriggerMs = 0;

  if (!sensor1.isBusy() && millis() - lastTriggerMs >= 100) {
    lastTriggerMs = millis();
    sensor1.trigger();
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

  // Other tasks can run here.
}
```

## Callback warning

The callback may be executed from interrupt context.

Keep the callback short.

Do not call these inside the callback:

```text
Serial.print()
delay()
Wire / I2C functions
SPI transfer
malloc / new
blocking functions
```

Recommended callback style:

```cpp
volatile bool distanceReady = false;
volatile float distanceCm = 0.0f;

void sr04Callback(const XC_SR04_Result &result) {
  if (result.status == XC_SR04_OK) {
    distanceCm = result.distanceCm;
    distanceReady = true;
  }
}
```

Then process the result in `loop()`:

```cpp
if (distanceReady) {
  distanceReady = false;
  Serial.println(distanceCm);
}
```

## Multiple sensors

The library supports multiple sensor objects, up to `XC_SR04_MAX_SENSORS`.

Example:

```cpp
XC_SR04 frontSensor;
XC_SR04 leftSensor;
XC_SR04 rightSensor;

frontSensor.begin(D8, D7, TIM2);
leftSensor.begin(D9, D6, TIM3);
rightSensor.begin(D10, D5, TIM4);
```

Each sensor needs its own timer instance in the current implementation.

### Important multi-sensor note

Triggering multiple ultrasonic sensors at the same time is not recommended.

Ultrasonic crosstalk can happen:

```text
Sensor A transmits sound
Sensor B receives Sensor A's echo
Result becomes wrong
```

For best reliability, trigger sensors sequentially:

```text
Trigger front sensor
Wait for callback
Small gap
Trigger left sensor
Wait for callback
Small gap
Trigger right sensor
```

The driver provides the building blocks, but the application should decide the trigger schedule.

## API summary

### `begin()`

```cpp
XC_SR04_Status begin(uint32_t trigPin,
                     uint32_t echoPin,
                     TIM_TypeDef *timerInstance);
```

Initializes the sensor.

Example:

```cpp
sensor.begin(D8, D7, TIM2);
```

### `trigger()`

```cpp
XC_SR04_Status trigger();
```

Starts one measurement.

Returns `XC_SR04_BUSY` if the previous measurement has not finished.

### `setCallback()`

```cpp
void setCallback(XC_SR04_Callback callback);
```

Registers a callback that receives result or timeout notification.

### `isBusy()`

```cpp
bool isBusy() const;
```

Returns true while a measurement is running.

### `setTimeoutUs()`

```cpp
void setTimeoutUs(uint32_t timeoutUs);
```

Sets timeout in microseconds.

Default is 30000 us.

### `statusToString()`

```cpp
static const char *statusToString(XC_SR04_Status status);
```

Converts status enum to readable text.

## Status codes

```cpp
XC_SR04_OK
XC_SR04_BUSY
XC_SR04_TIMEOUT
XC_SR04_INVALID_PIN
XC_SR04_INVALID_TIMER
XC_SR04_TIMER_IN_USE
XC_SR04_TOO_MANY_SENSORS
XC_SR04_NOT_INITIALIZED
XC_SR04_ALREADY_INITIALIZED
XC_SR04_ERROR
```

## Installation for local testing

Copy the library folder into:

```text
Documents/Arduino/libraries/XC_SR04
```

Expected structure:

```text
XC_SR04
├── library.properties
├── README.md
├── src
│   ├── XC_SR04.h
│   └── XC_SR04.cpp
└── examples
    ├── BasicSingleSensor
    │   └── BasicSingleSensor.ino
    └── MultipleSensors
        └── MultipleSensors.ino
```

Restart Arduino IDE.

Open:

```text
File → Examples → XC_SR04 → BasicSingleSensor
```

## Board tested

Initial test target:

```text
NUCLEO-G474RE
STM32duino / Arduino_Core_STM32
Arduino IDE 2.x
```

## Limitations

- Current implementation targets STM32duino only.
- Callback may run in interrupt context.
- Each sensor uses one timer instance.
- Simultaneous ultrasonic triggering is allowed by the driver but not recommended physically.
- Timer input capture backend is not implemented yet.

## Future improvements

Possible future versions:

- Sequential multi-sensor scheduler helper
- STM32 timer input capture backend
- Temperature compensation for speed of sound
- Optional callback dispatch outside ISR
- Support for other Arduino architectures
