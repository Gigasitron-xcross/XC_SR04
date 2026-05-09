/*
 * XC_SR04.h
 *
 * Object-oriented, non-blocking HC-SR04 / SR04 ultrasonic sensor driver
 * for STM32duino / Arduino_Core_STM32.
 *
 * Features:
 *   - No pulseIn()
 *   - Interrupt-driven ECHO measurement
 *   - HardwareTimer timeout service
 *   - Callback-based result notification
 *   - Supports up to XC_SR04_MAX_SENSORS sensor objects
 *
 * Important callback warning:
 *   The callback may be executed from interrupt context.
 *   Keep the callback short. Do not call Serial.print(), delay(), Wire, SPI,
 *   malloc/new, or other blocking functions inside the callback.
 *   Recommended usage: copy simple values and set a flag, then process the
 *   result in loop().
 *
 * Hardware warning:
 *   HC-SR04 ECHO is normally 5V when the module is powered from 5V.
 *   STM32 GPIO pins are normally 3.3V only. Use a voltage divider or level
 *   shifter on ECHO before connecting it to an STM32 pin.
 */

#ifndef XC_SR04_H
#define XC_SR04_H

#include <Arduino.h>

#ifndef ARDUINO_ARCH_STM32
#error "XC_SR04 currently requires STM32duino / Arduino_Core_STM32."
#endif

#include <HardwareTimer.h>

/*
 * Maximum number of SR04 sensor objects that can be registered.
 *
 * You may override this before including the library:
 *   #define XC_SR04_MAX_SENSORS 4
 *   #include <XC_SR04.h>
 *
 * Keeping this limit small helps control RAM usage and interrupt service time.
 */
#ifndef XC_SR04_MAX_SENSORS
#define XC_SR04_MAX_SENSORS 8
#endif

#if (XC_SR04_MAX_SENSORS < 1)
#error "XC_SR04_MAX_SENSORS must be at least 1."
#endif

#if (XC_SR04_MAX_SENSORS > 8)
#error "This implementation supports a maximum of 8 sensors."
#endif

/*
 * XC_SR04_Status
 *
 * Return and result status codes.
 */
enum XC_SR04_Status {
  XC_SR04_OK = 0,
  XC_SR04_BUSY,
  XC_SR04_TIMEOUT,
  XC_SR04_INVALID_PIN,
  XC_SR04_INVALID_TIMER,
  XC_SR04_TIMER_IN_USE,
  XC_SR04_TOO_MANY_SENSORS,
  XC_SR04_NOT_INITIALIZED,
  XC_SR04_ALREADY_INITIALIZED,
  XC_SR04_ERROR
};

/*
 * XC_SR04_Result
 *
 * Passed to the user callback when a measurement completes or fails.
 *
 * durationUs:
 *   Echo pulse duration in microseconds. Valid only when status == XC_SR04_OK.
 *
 * distanceCm:
 *   Calculated distance in centimeters. Valid only when status == XC_SR04_OK.
 *   For error/timeout cases, distanceCm is set to -1.0f.
 */
struct XC_SR04_Result {
  XC_SR04_Status status;
  uint8_t sensorId;
  uint32_t durationUs;
  float distanceCm;
};

/*
 * User callback function type.
 *
 * Note:
 *   The callback may be called from interrupt context.
 *   Keep it short and ISR-safe.
 */
typedef void (*XC_SR04_Callback)(const XC_SR04_Result &result);

/*
 * XC_SR04
 *
 * Each object represents one ultrasonic sensor.
 *
 * Basic usage:
 *   XC_SR04 sensor;
 *   sensor.begin(D8, D7, TIM2);
 *   sensor.setCallback(myCallback);
 *   sensor.trigger();
 */
class XC_SR04 {
public:
  XC_SR04();

  /*
   * begin()
   *
   * Initializes one SR04 sensor.
   *
   * trigPin:
   *   Arduino/STM32duino pin used to drive the TRIG input.
   *
   * echoPin:
   *   Arduino/STM32duino pin used to read the ECHO output.
   *   This pin must support external interrupt.
   *
   * timerInstance:
   *   STM32 timer instance used as the timeout/timebase service.
   *   Example: TIM2, TIM3, TIM4.
   *
   * Return:
   *   XC_SR04_OK on success, otherwise an error status.
   */
  XC_SR04_Status begin(uint32_t trigPin,
                       uint32_t echoPin,
                       TIM_TypeDef *timerInstance);

  /*
   * trigger()
   *
   * Starts one measurement.
   *
   * This function generates the 10 us TRIG pulse and then returns quickly.
   * The measurement result is later returned through the callback.
   *
   * Return:
   *   XC_SR04_OK if measurement started.
   *   XC_SR04_BUSY if a previous measurement is still in progress.
   */
  XC_SR04_Status trigger();

  /*
   * isBusy()
   *
   * Returns true while a measurement is in progress.
   */
  bool isBusy() const;

  /*
   * isInitialized()
   *
   * Returns true after begin() succeeds.
   */
  bool isInitialized() const;

  /*
   * setCallback()
   *
   * Registers a result callback.
   *
   * The callback may be called from interrupt context.
   */
  void setCallback(XC_SR04_Callback callback);

  /*
   * setTimeoutUs()
   *
   * Sets echo timeout in microseconds.
   *
   * Default: 30000 us.
   * This is around 5 m maximum range, depending on air conditions.
   */
  void setTimeoutUs(uint32_t timeoutUs);

  /*
   * getTimeoutUs()
   *
   * Returns current timeout in microseconds.
   */
  uint32_t getTimeoutUs() const;

  /*
   * getSensorId()
   *
   * Returns internal sensor ID, from 0 to XC_SR04_MAX_SENSORS - 1.
   * Returns 255 if not initialized.
   */
  uint8_t getSensorId() const;

  /*
   * statusToString()
   *
   * Converts status enum to readable text.
   */
  static const char *statusToString(XC_SR04_Status status);

private:
  enum State {
    STATE_IDLE = 0,
    STATE_WAIT_RISING,
    STATE_WAIT_FALLING
  };

private:
  uint32_t _trigPin;
  uint32_t _echoPin;
  TIM_TypeDef *_timerInstance;
  HardwareTimer *_timer;

  volatile bool _initialized;
  volatile bool _busy;
  volatile State _state;

  volatile uint32_t _startUs;
  volatile uint32_t _endUs;

  uint32_t _timeoutUs;
  uint8_t _sensorId;

  XC_SR04_Callback _callback;

private:
  void handleEchoEdgeISR();
  void handleTimeoutISR();

  void finishFromISR(XC_SR04_Status status, uint32_t durationUs);

  static XC_SR04 *_instances[8];
  static uint8_t _instanceCount;

  static bool isTimerAlreadyUsed(TIM_TypeDef *timerInstance);

  static void echoThunk0();
  static void echoThunk1();
  static void echoThunk2();
  static void echoThunk3();
  static void echoThunk4();
  static void echoThunk5();
  static void echoThunk6();
  static void echoThunk7();

  static void timerThunk0();
  static void timerThunk1();
  static void timerThunk2();
  static void timerThunk3();
  static void timerThunk4();
  static void timerThunk5();
  static void timerThunk6();
  static void timerThunk7();

  static void (*const _echoThunks[8])();
  static void (*const _timerThunks[8])();
};

#endif
