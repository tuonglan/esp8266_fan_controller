/*************************************************************
  Blynk is a platform with iOS and Android apps to control
  ESP32, Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build mobile and web interfaces for any
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: https://www.blynk.io
    Sketch generator:           https://examples.blynk.cc
    Blynk community:            https://community.blynk.cc
    Follow us:                  https://www.fb.com/blynkapp
                                https://twitter.com/blynk_app

  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/

// Template ID
#define BLYNK_TEMPLATE_ID "TMPL6PyORnvfe" // Template ID
#define BLYNK_TEMPLATE_NAME "Home Fan Controller" // Device Name
//#define BLYNK_AUTH_TOKEN "" // Authentication number
#define BLYNK_FIRMWARE_VERSION "0.1.3"

#define LOCAL_WIFI_SSID "" //"Wifi SSID"
#define LOCAL_WIFI_PASS "" //"Wifi Pass"

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial

// Edgent configuration
#define APP_DEBUG
#define USE_NODE_MCU_BOARD

//#include <ESP8266WiFi.h>
//#include <BlynkSimpleEsp8266.h>
#include "BlynkEdgent.h"

#include "fan_settings.h"
#include "fan_controller.hpp"
#include "stepper_motor_controller.hpp"


// ******************************************************************************************************
// ***********************************------ Blynk Initalize ---*****************************************
StepperMotor stepper_motor(&Serial, &Blynk);
FanController fan_controller(&Serial, &Blynk, &stepper_motor);

BLYNK_WRITE_DEFAULT() {
  uint8_t pin = (uint8_t)request.pin;

#ifdef APP_DEBUG
  Serial.print("Handling virutalPin: ");
  Serial.println(pin);
  Serial.print("Blynk State: ");
  Serial.println(BlynkState::get());
#endif

  // NOTE: Only virtualWRite whne BlynkState == MODE_RUNNING
  // This is only a safe-guard
  if (BlynkState::get() == MODE_RUNNING) {
    if (pin >= FanControllerVPins::ON_OFF && pin <= FanControllerVPins::RESERVED)
      fan_controller.vpin_handler(pin, &param);
    else if (pin >= StepperMotorVPins::ON_OFF && pin <= StepperMotorVPins::RESERVED)
      stepper_motor.vpin_handler(pin, &param);
  }
}


// NOTE: Not necessary anymore when using BlynkEdgent
// Now init the node at running for the first time
//BLYNK_CONNECTED {
void init_values() {
  Blynk.syncVirtual(FanControllerVPins::ON_OFF);
  Blynk.syncVirtual(FanControllerVPins::SPEED);
  Blynk.syncVirtual(FanControllerVPins::TIMER_ON_OFF);
  Blynk.syncVirtual(FanControllerVPins::TIMER_LENGTH);
  Blynk.syncVirtual(FanControllerVPins::TIMER_COUNTDOWN);
  Blynk.syncVirtual(FanControllerVPins::VERSION);
  Blynk.syncVirtual(FanControllerVPins::RESERVED);

  Blynk.syncVirtual(StepperMotorVPins::ON_OFF);
  Blynk.syncVirtual(StepperMotorVPins::SPEED);
  Blynk.syncVirtual(StepperMotorVPins::MAX_ANGLE);
  Blynk.syncVirtual(StepperMotorVPins::CURRENT_ANGLE);
  Blynk.syncVirtual(StepperMotorVPins::RESET_ANGLE);
  Blynk.syncVirtual(StepperMotorVPins::RESERVED);
}

// ******************************************************************************************************
// ***********************************------- Main functions ---*****************************************
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // ESP01/ESP01S
//  Serial.begin(9600);   // ESP8266 MCU
//  Serial.setDebugOutput(true);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(SM_PIN0, OUTPUT);
  pinMode(SM_PIN1, OUTPUT);
  pinMode(SM_PIN2, OUTPUT);
  pinMode(SM_PIN3, OUTPUT);

//  Blynk.begin(BLYNK_AUTH_TOKEN, LOCAL_WIFI_SSID, LOCAL_WIFI_PASS);
  BlynkEdgent.begin();
}

void loop() {
  static bool devices_inited = false;
  // put your main code here, to run repeatedly:
//  Blynk.run();
  BlynkEdgent.run();

  if (BlynkState::get() == MODE_RUNNING) {
    if (!devices_inited) {
      fan_controller.init();
      stepper_motor.init();

      Serial.println("Initing the values when after resetting the device...");
      init_values();
      devices_inited = true;
    }

    fan_controller.run();
    stepper_motor.run();
  }
}
