/*
 * XC_SR04.cpp
 *
 * Object-oriented, non-blocking HC-SR04 / SR04 ultrasonic sensor driver
 * for STM32duino / Arduino_Core_STM32.
 *
 * This driver avoids pulseIn(). It uses:
 *   - external interrupt on ECHO pin
 *   - HardwareTimer for timeout and microsecond time reference
 *   - callback to notify the user when result/error is ready
 *
 * Callback warning:
 *   The user callback may execute from interrupt context.
 *   Keep the callback short and only set flags or copy simple values.
 */

#include "XC_SR04.h"

/*
 * Static registry.
 *
 * attachInterrupt() and HardwareTimer attachInterrupt() require plain function
 * pointers, not normal C++ member functions. The thunk functions below map
 * each interrupt back to the correct XC_SR04 object instance.
 */
XC_SR04 *XC_SR04::_instances[8] = { nullptr };
uint8_t XC_SR04::_instanceCount = 0;

void (*const XC_SR04::_echoThunks[8])() = {
  XC_SR04::echoThunk0,
  XC_SR04::echoThunk1,
  XC_SR04::echoThunk2,
  XC_SR04::echoThunk3,
  XC_SR04::echoThunk4,
  XC_SR04::echoThunk5,
  XC_SR04::echoThunk6,
  XC_SR04::echoThunk7
};

void (*const XC_SR04::_timerThunks[8])() = {
  XC_SR04::timerThunk0,
  XC_SR04::timerThunk1,
  XC_SR04::timerThunk2,
  XC_SR04::timerThunk3,
  XC_SR04::timerThunk4,
  XC_SR04::timerThunk5,
  XC_SR04::timerThunk6,
  XC_SR04::timerThunk7
};

XC_SR04::XC_SR04()
  : _trigPin(0),
    _echoPin(0),
    _timerInstance(nullptr),
    _timer(nullptr),
    _initialized(false),
    _busy(false),
    _state(STATE_IDLE),
    _startUs(0),
    _endUs(0),
    _timeoutUs(30000),
    _sensorId(255),
    _callback(nullptr) {
}

XC_SR04_Status XC_SR04::begin(uint32_t trigPin,
                              uint32_t echoPin,
                              TIM_TypeDef *timerInstance) {
  if (_initialized) {
    return XC_SR04_ALREADY_INITIALIZED;
  }

  if (timerInstance == nullptr) {
    return XC_SR04_INVALID_TIMER;
  }

  /*
   * Check whether echoPin supports interrupt when the core provides
   * NOT_AN_INTERRUPT.
   */
#ifdef NOT_AN_INTERRUPT
  if (digitalPinToInterrupt(echoPin) == NOT_AN_INTERRUPT) {
    return XC_SR04_INVALID_PIN;
  }
#endif

  noInterrupts();

  if (_instanceCount >= XC_SR04_MAX_SENSORS) {
    interrupts();
    return XC_SR04_TOO_MANY_SENSORS;
  }

  if (isTimerAlreadyUsed(timerInstance)) {
    interrupts();
    return XC_SR04_TIMER_IN_USE;
  }

  _sensorId = _instanceCount;
  _instances[_sensorId] = this;
  _instanceCount++;

  interrupts();

  _trigPin = trigPin;
  _echoPin = echoPin;
  _timerInstance = timerInstance;

  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  digitalWrite(_trigPin, LOW);

  /*
   * Create a timer object for this sensor.
   *
   * The timer is used as:
   *   - a microsecond counter while a measurement is active
   *   - a timeout source if no valid echo is received
   */
  _timer = new HardwareTimer(_timerInstance);

  if (_timer == nullptr) {
    return XC_SR04_INVALID_TIMER;
  }

  _timer->pause();
  _timer->setOverflow(_timeoutUs, MICROSEC_FORMAT);
  _timer->setCount(0, MICROSEC_FORMAT);
  _timer->attachInterrupt(_timerThunks[_sensorId]);

  attachInterrupt(digitalPinToInterrupt(_echoPin),
                  _echoThunks[_sensorId],
                  CHANGE);

  _busy = false;
  _state = STATE_IDLE;
  _initialized = true;

  return XC_SR04_OK;
}

XC_SR04_Status XC_SR04::trigger() {
  if (!_initialized) {
    return XC_SR04_NOT_INITIALIZED;
  }

  if (_busy) {
    return XC_SR04_BUSY;
  }

  /*
   * Prepare state before sending trigger pulse.
   */
  noInterrupts();
  _busy = true;
  _state = STATE_WAIT_RISING;
  _startUs = 0;
  _endUs = 0;
  interrupts();

  /*
   * Start timer from zero.
   *
   * If no valid echo completes before _timeoutUs, the timer interrupt calls
   * handleTimeoutISR().
   */
  _timer->pause();
  _timer->setOverflow(_timeoutUs, MICROSEC_FORMAT);
  _timer->setCount(0, MICROSEC_FORMAT);
  _timer->refresh();
  _timer->resume();

  /*
   * HC-SR04 trigger requirement:
   *   TRIG high pulse of at least 10 us.
   */
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  return XC_SR04_OK;
}

bool XC_SR04::isBusy() const {
  return _busy;
}

bool XC_SR04::isInitialized() const {
  return _initialized;
}

void XC_SR04::setCallback(XC_SR04_Callback callback) {
  _callback = callback;
}

void XC_SR04::setTimeoutUs(uint32_t timeoutUs) {
  /*
   * Avoid very small timeout values that may fire before ECHO can complete.
   */
  if (timeoutUs < 1000) {
    timeoutUs = 1000;
  }

  _timeoutUs = timeoutUs;

  if (_timer != nullptr) {
    _timer->pause();
    _timer->setOverflow(_timeoutUs, MICROSEC_FORMAT);
    _timer->setCount(0, MICROSEC_FORMAT);
  }
}

uint32_t XC_SR04::getTimeoutUs() const {
  return _timeoutUs;
}

uint8_t XC_SR04::getSensorId() const {
  return _sensorId;
}

void XC_SR04::handleEchoEdgeISR() {
  if (!_busy) {
    return;
  }

  /*
   * Read timer count as close as possible to the edge event.
   */
  uint32_t nowUs = _timer->getCount(MICROSEC_FORMAT);
  int level = digitalRead(_echoPin);

  if (level == HIGH) {
    /*
     * Rising edge: ECHO pulse started.
     */
    if (_state == STATE_WAIT_RISING) {
      _startUs = nowUs;
      _state = STATE_WAIT_FALLING;
    }
  } else {
    /*
     * Falling edge: ECHO pulse ended.
     */
    if (_state == STATE_WAIT_FALLING) {
      _endUs = nowUs;

      uint32_t durationUs = 0;

      if (_endUs >= _startUs) {
        durationUs = _endUs - _startUs;
      } else {
        /*
         * This should not normally happen because the timer is restarted
         * for each measurement and timeout occurs before overflow wrapping.
         */
        durationUs = 0;
      }

      finishFromISR(XC_SR04_OK, durationUs);
    }
  }
}

void XC_SR04::handleTimeoutISR() {
  if (!_busy) {
    return;
  }

  finishFromISR(XC_SR04_TIMEOUT, 0);
}

void XC_SR04::finishFromISR(XC_SR04_Status status, uint32_t durationUs) {
  if (_timer != nullptr) {
    _timer->pause();
    _timer->setCount(0, MICROSEC_FORMAT);
  }

  _busy = false;
  _state = STATE_IDLE;

  XC_SR04_Result result;
  result.status = status;
  result.sensorId = _sensorId;
  result.durationUs = durationUs;

  if (status == XC_SR04_OK) {
    /*
     * Speed of sound around room temperature:
     *   343 m/s = 0.0343 cm/us
     *
     * Divide by 2 because the sound travels:
     *   sensor -> object -> sensor
     */
    result.distanceCm = durationUs * 0.0343f / 2.0f;
  } else {
    result.distanceCm = -1.0f;
  }

  if (_callback != nullptr) {
    _callback(result);
  }
}

bool XC_SR04::isTimerAlreadyUsed(TIM_TypeDef *timerInstance) {
  for (uint8_t i = 0; i < _instanceCount; i++) {
    if (_instances[i] != nullptr &&
        _instances[i]->_timerInstance == timerInstance) {
      return true;
    }
  }

  return false;
}

const char *XC_SR04::statusToString(XC_SR04_Status status) {
  switch (status) {
    case XC_SR04_OK:
      return "OK";

    case XC_SR04_BUSY:
      return "BUSY";

    case XC_SR04_TIMEOUT:
      return "TIMEOUT";

    case XC_SR04_INVALID_PIN:
      return "INVALID_PIN";

    case XC_SR04_INVALID_TIMER:
      return "INVALID_TIMER";

    case XC_SR04_TIMER_IN_USE:
      return "TIMER_IN_USE";

    case XC_SR04_TOO_MANY_SENSORS:
      return "TOO_MANY_SENSORS";

    case XC_SR04_NOT_INITIALIZED:
      return "NOT_INITIALIZED";

    case XC_SR04_ALREADY_INITIALIZED:
      return "ALREADY_INITIALIZED";

    case XC_SR04_ERROR:
    default:
      return "ERROR";
  }
}

/*
 * Echo interrupt thunks.
 */
void XC_SR04::echoThunk0() { if (_instances[0]) _instances[0]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk1() { if (_instances[1]) _instances[1]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk2() { if (_instances[2]) _instances[2]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk3() { if (_instances[3]) _instances[3]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk4() { if (_instances[4]) _instances[4]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk5() { if (_instances[5]) _instances[5]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk6() { if (_instances[6]) _instances[6]->handleEchoEdgeISR(); }
void XC_SR04::echoThunk7() { if (_instances[7]) _instances[7]->handleEchoEdgeISR(); }

/*
 * Timer timeout thunks.
 */
void XC_SR04::timerThunk0() { if (_instances[0]) _instances[0]->handleTimeoutISR(); }
void XC_SR04::timerThunk1() { if (_instances[1]) _instances[1]->handleTimeoutISR(); }
void XC_SR04::timerThunk2() { if (_instances[2]) _instances[2]->handleTimeoutISR(); }
void XC_SR04::timerThunk3() { if (_instances[3]) _instances[3]->handleTimeoutISR(); }
void XC_SR04::timerThunk4() { if (_instances[4]) _instances[4]->handleTimeoutISR(); }
void XC_SR04::timerThunk5() { if (_instances[5]) _instances[5]->handleTimeoutISR(); }
void XC_SR04::timerThunk6() { if (_instances[6]) _instances[6]->handleTimeoutISR(); }
void XC_SR04::timerThunk7() { if (_instances[7]) _instances[7]->handleTimeoutISR(); }
