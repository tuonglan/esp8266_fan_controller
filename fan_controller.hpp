#ifndef _FAN_CONTROLLER_HPP_
#define _FAN_CONTROLLER_HPP_

#include "fan_settings.h"
#include "stepper_motor_controller.hpp"


// Pins for controlling fan
// #0: ON/OFF
// #1: Fan speed
// #2: Timer ON/OFF
// #3: Timer interval
// #4: Timer countdown
// #5: Reserved
class FanControllerVPins {
 public:
  enum VPins {
    ON_OFF = 0,
    SPEED = 1,
    TIMER_ON_OFF = 2,
    TIMER_LENGTH = 3,
    TIMER_COUNTDOWN = 4,
    VERSION = 5,
    RESERVED = 6
  };

  static size_t const vpin_count = 7;
};

void update_timer_func(void *fc);


// ******************************************************************************************************
// ***********************************------- Fain Controller ---*****************************************
class FanController {
 public:
  FanController(HardwareSerial *serial, BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *blynk, StepperMotor *s_motor);
  ~FanController();

  void init();
  void run();
  void vpin_handler(uint8_t pin, BlynkParam const *param);
  void update_timer();
 protected:
  void run_timer();
  
  void switch_on_handler(BlynkParam const *param);
  void speed_handler(BlynkParam const *param);
  void timer_on_handler(BlynkParam const *param);
  void timer_time_handler(BlynkParam const *param);

 private:
  void _start_fan(bool update_blynk=false);
  void _stop_fan(bool update_blynk=false);
  void _set_timer();
  void _unset_timer(bool update_blynk=false);
  void _set_speed();

  static uint8_t const MIN_SPEED = 32;
  static uint8_t const MAX_SPEED = 255;

//  uint8_t *_vpins;
//  size_t _n_vpins;

  StepperMotor *_s_motor;
  HardwareSerial *_serial;
  BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *_blynk;
  BlynkTimer *_timer;
  int _timer_id;

  // Stream data
  int _blk_switch_on;
  uint8_t _blk_speed;
  int _blk_timer_on;
  int _blk_timer_time;

  // Data for status
  int _remaining_time;
  uint32_t _timer_start_ts_ms;
};


FanController::FanController(HardwareSerial *serial, BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *blynk, StepperMotor *s_motor)
  : _serial(serial), _blynk(blynk), _s_motor(s_motor) {

  // Init vpins
//  _n_vpins = sizeof(vpins) / sizeof(uint8_t);
//  _vpins = new uint8_t[_n_vpins];
//  for (int i=0;i<_n_vpins;++i)
//    _vpins[i] = vpins[i];

  // Init timer
  _timer = new BlynkTimer();
  _timer_id = -1;
}


FanController::~FanController() {
//  delete[] _vpins;
  delete _timer;
}


void FanController::init() {
  this->_stop_fan(true);
  this->_unset_timer(true);

  this->_blynk->virtualWrite(FanControllerVPins::VERSION, BLYNK_FIRMWARE_VERSION);
}


void FanController::run() {
  this->run_timer();
}


void FanController::run_timer() {
  if (_timer_id >= 0)
    _timer->run();
}


void FanController::vpin_handler(uint8_t pin, BlynkParam const *param) {
  switch(pin) {
    case FanControllerVPins::ON_OFF :
      this->switch_on_handler(param);
      break;
    case FanControllerVPins::SPEED :
      this->speed_handler(param);
      break;
    case FanControllerVPins::TIMER_ON_OFF :
      this->timer_on_handler(param);
      break;
    case FanControllerVPins::TIMER_LENGTH :
      this->timer_time_handler(param);
      break;
    default:
      break;
  }
}


void FanController::update_timer() {
  uint32_t current_ts_ms = millis();
  uint32_t elapse = 0;

//  this->_serial->print("Updating timer...\nCurrent ts_ms: ");
//  this->_serial->println(current_ts_ms);
//  this->_serial->print("Current remaining time: ");
//  this->_serial->println(_remaining_time);

  // Calculate the elapse
  if (current_ts_ms < _timer_start_ts_ms) {
    elapse = ULONG_MAX_VALUE - _timer_start_ts_ms + current_ts_ms;
  }
  else
    elapse = current_ts_ms - _timer_start_ts_ms;

  // Update timer
  if (elapse > 60000) {
    _timer_start_ts_ms = current_ts_ms + (elapse % 60000);
    _remaining_time -= elapse / 60000;

    if (_remaining_time <= 0) {
      _remaining_time = 0;
      this->_stop_fan(true);
      this->_unset_timer(true);
    }
    else
      // Update the timer
      this->_blynk->virtualWrite(FanControllerVPins::TIMER_COUNTDOWN, _remaining_time);
  }
}


void FanController::switch_on_handler(BlynkParam const *param) {
  int value = param->asInt();
  if (value == 0) {
    this->_stop_fan();

    this->_unset_timer(true);
  }
  else if (value == 1) {
    this->_start_fan();

    if (_blk_timer_on == 1)
      this->_set_timer();
  }

  this->_serial->print("On/Off switch: ");
  this->_serial->println(value);
}


void FanController::speed_handler(BlynkParam const *param) {
  int value = param->asInt();
  if (value < FanController::MIN_SPEED) {
    value = FanController::MIN_SPEED;
    this->_blynk->virtualWrite(FanControllerVPins::SPEED, value);
  }
  if (value > FanController::MAX_SPEED) {
    value = FanController::MAX_SPEED;
    this->_blynk->virtualWrite(FanControllerVPins::SPEED, value);
  }

  _blk_speed = value;
  if (_blk_switch_on == 1)
    this->_set_speed();
}


void FanController::timer_on_handler(BlynkParam const *param) {
  int value = param->asInt();
  if (value == 0 && _blk_timer_on == 1) {
    this->_unset_timer();
  }
  else if (value == 1 && _blk_timer_on == 0) {
    this->_start_fan(true);
    this->_set_timer();
  }
}


void FanController::timer_time_handler(BlynkParam const *param) {
  int value = param->asInt();
  _blk_timer_time = value;
}


void FanController::_start_fan(bool update_blynk) {
  analogWrite(PWM_PIN, _blk_speed);
  _blk_switch_on = 1;

  if (update_blynk)
    this->_blynk->virtualWrite(FanControllerVPins::ON_OFF, 1);
}


void FanController::_stop_fan(bool update_blynk) {
  analogWrite(PWM_PIN, 0);
  _blk_switch_on = 0;
  _s_motor->stop();

  if (update_blynk)
    this->_blynk->virtualWrite(FanControllerVPins::ON_OFF, 0);
}


void FanController::_set_timer() {
  if (_timer_id >= 0)
    this->_unset_timer();

  _blk_timer_on = 1;
  _timer_id = this->_timer->setInterval(1000, update_timer_func, this);
  _remaining_time = _blk_timer_time;
  _timer_start_ts_ms = millis();

  // Update blynk count down
  this->_blynk->virtualWrite(FanControllerVPins::TIMER_COUNTDOWN, _remaining_time);

  this->_serial->print("Timer set: ");
  this->_serial->println(_timer_id);
}


void FanController::_unset_timer(bool update_blynk) {
  int timer_id = _timer_id;

  if (_timer_id >= 0) {
    this->_timer->deleteTimer(_timer_id);
    _timer_id = -1;
  }
  _remaining_time = 0;
  _blk_timer_on = 0;

  // BUGGG HERE ==> Can't write Pin whne BlynkState is not RUNNING
  // Update blynk count down
  this->_blynk->virtualWrite(FanControllerVPins::TIMER_COUNTDOWN, _remaining_time);
  if (update_blynk)
    this->_blynk->virtualWrite(FanControllerVPins::TIMER_ON_OFF, 0);

  this->_serial->print("Timer unset: ");
  this->_serial->println(timer_id);
}


void FanController::_set_speed() {
  analogWrite(PWM_PIN, _blk_speed);
}


void update_timer_func(void *p_fc) {
  FanController *fc = (FanController *)p_fc;
  fc->update_timer();
}


#endif
