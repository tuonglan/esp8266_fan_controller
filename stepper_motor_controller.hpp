#ifndef _STEPPER_MOTOR_CONTROLLER_HPP_
#define _STEPPER_MOTOR_CONTROLLER_HPP_

#include "fan_settings.h"


class StepperMotorVPins {
 public:
  enum VPins {
    ON_OFF = 10,
    SPEED = 11,
    MAX_ANGLE = 12,
    CURRENT_ANGLE = 13,
    RESET_ANGLE = 14,
    RESERVED = 15,
  };

  static size_t const vpin_count = 6;
};
//uint8_t const FAN_BLYNK_VPINS[] = {0, 1, 2, 3, 4, 5};
//size_t const FAN_BLYNK_VPINS_N = sizeof(FAN_BLYNK_VPINS) / sizeof(uint8_t);

void update_step_func(void *sm);

void sm_step_func1(uint16_t step_idx);
void sm_step_func2(uint16_t step_idx);


// ******************************************************************************************************
// ***********************************------- Fain Controller ---*****************************************
class StepperMotor {
 public:
  StepperMotor(HardwareSerial *serial, BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *blynk);
  ~StepperMotor();

  void init();
  void run();
  void vpin_handler(uint8_t pin, BlynkParam const *param);
  void step();
  void stop() { this->_unset_timer(true); }
 protected:
  void switch_on_handler(BlynkParam const *param);
  void speed_handler(BlynkParam const *param);
  void max_angle_handler(BlynkParam const *param);
  void current_angle_handler(BlynkParam const *param);
  void reset_angle_handler(BlynkParam const *param);

 private:
  void _step(uint16_t step_idx);
  void _set_speed(uint16_t speed) { _blk_speed = speed; }
  void _set_max_angle(uint16_t max_angle) { _blk_max_angle = max_angle; }
  void _set_current_angle(int16_t angle) { _blk_current_angle = angle; }
  void _reset_angle();

  void _set_timer(bool update_blynk=false);
  void _unset_timer(bool update_blynk=false);

  uint8_t _blk_switch_on;
  uint16_t _blk_speed;
  uint16_t _blk_max_angle;      // Measured in number of steps
  int16_t _blk_current_angle;  // Measured in number of steps

  bool _direction;              // True for clockwise and FAlse for counterclockwise
  uint32_t _step_ts_ms;

  int _timer_id;

  HardwareSerial *_serial;
  BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *_blynk;
  BlynkTimer *_timer;
};


StepperMotor::StepperMotor(HardwareSerial *serial, BlynkWifi<BlynkArduinoClientSecure<WiFiClientSecure>> *blynk) 
  : _blynk(blynk), _serial(serial)
  , _timer_id(-1), _blk_current_angle(0), _direction(true)
  , _blk_speed(100) {

  _timer = new BlynkTimer();
}


StepperMotor::~StepperMotor() {
  delete _timer;
}


void StepperMotor::init() {
  this->_unset_timer(true);
}


void StepperMotor::run() {
  if (_timer_id >= 0)
    _timer->run();
}


void StepperMotor::vpin_handler(uint8_t pin, BlynkParam const *param) {
  if (StepperMotorVPins::ON_OFF == pin)
    this->switch_on_handler(param);
  else if (StepperMotorVPins::SPEED == pin)
    this->speed_handler(param);
  else if (StepperMotorVPins::MAX_ANGLE == pin)
    this->max_angle_handler(param);
  else if (StepperMotorVPins::CURRENT_ANGLE == pin)
    this->current_angle_handler(param);
  else if (StepperMotorVPins::RESET_ANGLE == pin)
    this->reset_angle_handler(param);
}


void StepperMotor::step() {
  // Decide the direction
  if (_blk_current_angle >= _blk_max_angle) {
    _blk_current_angle = _blk_max_angle;
    _direction = false;
  }
  else if (_blk_current_angle <= 0) {
    _blk_current_angle = 0;
    _direction = true;
  }

  // Increase or decrease angle
  if (_direction)
    _blk_current_angle++;
  else
    _blk_current_angle--;

  // Rotate 
  this->_step(_blk_current_angle);

  // Update current angle if neccessary
  uint32_t current_ts_ms = millis();
  int elapse = 0;

  if (current_ts_ms < _step_ts_ms)
    elapse = ULONG_MAX_VALUE - _step_ts_ms + current_ts_ms;
  else
    elapse = current_ts_ms - _step_ts_ms;
  if (elapse >= 1000) {
    this->_blynk->virtualWrite(StepperMotorVPins::CURRENT_ANGLE, _blk_current_angle);
    _step_ts_ms = current_ts_ms;
  }

//  this->_serial->print("The current angle: ");
//  this->_serial->println(_blk_current_angle);
//  this->_serial->print("Elapse: ");
//  this->_serial->println(elapse);
}


void StepperMotor::switch_on_handler(BlynkParam const *param) {
  int value = param->asInt();
  if (value == 0) {
    this->_unset_timer();
    if (_blk_switch_on == 1)
      this->_blynk->virtualWrite(StepperMotorVPins::CURRENT_ANGLE, _blk_current_angle);
    _blk_switch_on = 0;
  }
  else if (value == 1) {
    this->_set_timer();
    _blk_switch_on = 1;
  }

  this->_serial->print("Rotation On/Off: ");
  this->_serial->println(value);
}


void StepperMotor::speed_handler(BlynkParam const *param) {
  int value = param->asInt();
  this->_set_speed(value);

  this->_serial->print("Rotation Speed set at: ");
  this->_serial->println(value);
}


void StepperMotor::max_angle_handler(BlynkParam const *param) {
  int value = param->asInt();
  this->_set_max_angle(value);
}


void StepperMotor::current_angle_handler(BlynkParam const *param) {
  int value = param->asInt();
  this->_set_current_angle(value);
}


void StepperMotor::reset_angle_handler(BlynkParam const *param) {
  int value = param->asInt();
  if (value == 1) {
    this->_reset_angle();
    this->_blynk->virtualWrite(StepperMotorVPins::RESET_ANGLE, 0);
  }
}


void StepperMotor::_reset_angle() {
  _blk_current_angle = 0;
  _direction = true;
  this->_blynk->virtualWrite(StepperMotorVPins::CURRENT_ANGLE, 0);
}


void StepperMotor::_set_timer(bool update_blynk) {
  if (_timer_id >= 0)
    this->_unset_timer();

  int interval = 1000 / _blk_speed;
  if (interval < SM_MIN_DELAY)
    interval = SM_MIN_DELAY;
  _step_ts_ms = millis();
  _timer_id = this->_timer->setInterval(interval, update_step_func, this);

  if (update_blynk)
    this->_blynk->virtualWrite(StepperMotorVPins::ON_OFF, 1);
}


void StepperMotor::_unset_timer(bool update_blynk) {
  if (_timer_id >= 0) {
    this->_timer->deleteTimer(_timer_id);
    _timer_id = -1;
  }

  // Stop the motor
  digitalWrite(SM_PIN0, LOW);
  digitalWrite(SM_PIN1, LOW);
  digitalWrite(SM_PIN2, LOW);
  digitalWrite(SM_PIN3, LOW);

  if (update_blynk)
    this->_blynk->virtualWrite(StepperMotorVPins::ON_OFF, 0);
}


void StepperMotor::_step(uint16_t step_idx) {
  sm_step_func1(step_idx);
}


void sm_step_func2(uint16_t step_idx) {
  uint8_t phase = step_idx % 4;
  switch(phase) {
    case 0:
      digitalWrite(SM_PIN0, HIGH); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, HIGH); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 1:
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, HIGH); 
      digitalWrite(SM_PIN2, HIGH); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 2:
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, HIGH); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, HIGH); 
      break; 
    case 3:
      digitalWrite(SM_PIN0, HIGH); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, HIGH); 
      break; 
    default:
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
  }
}


void sm_step_func1(uint16_t step_idx) {
  uint8_t phase = step_idx % 8;
  switch(phase) { 
    case 0: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, HIGH); 
      break; 
    case 1: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, HIGH); 
      digitalWrite(SM_PIN3, HIGH); 
      break; 
    case 2: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, HIGH); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 3: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, HIGH); 
      digitalWrite(SM_PIN2, HIGH); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 4: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, HIGH); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 5: 
      digitalWrite(SM_PIN0, HIGH); 
      digitalWrite(SM_PIN1, HIGH); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 6: 
      digitalWrite(SM_PIN0, HIGH); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
    case 7: 
      digitalWrite(SM_PIN0, HIGH); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, HIGH); 
      break; 
    default: 
      digitalWrite(SM_PIN0, LOW); 
      digitalWrite(SM_PIN1, LOW); 
      digitalWrite(SM_PIN2, LOW); 
      digitalWrite(SM_PIN3, LOW); 
      break; 
  }
}


void update_step_func(void *p_sm) {
  StepperMotor *sm = (StepperMotor *)p_sm;
  sm->step();
}


#endif
