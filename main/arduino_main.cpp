/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>


#define IN1R 12
#define IN2R 14
#define IN1L 13
#define IN2L 15

Servo servo;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}


// Arduino setup function. Runs in CPU 1
void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
 
    servo.attach(16);

    // motor controller outputs
    pinMode(IN1R, OUTPUT);
    pinMode(IN2R, OUTPUT);
    pinMode(IN1L, OUTPUT);
    pinMode(IN2L, OUTPUT);

    Serial.begin(115200);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
           
            if (controller->l1() == 1) {
                Serial.print("Servo up");
                servo.write(1000);
            }
            if (controller->r1() == 1) {
                Serial.print("Servo down");
                servo.write(2000);
            }
            if (controller->l1() == 0 && controller->r1() == 0) {
                Serial.print("Servo stop");
                servo.write(1500);
            }

            if(controller->axisRY() > 0) { // negative y is upward on stick
                Serial.println(" DC motor move forward");
                digitalWrite(IN1R, LOW);
                digitalWrite(IN2R, HIGH);
                digitalWrite(IN1L, LOW);
                digitalWrite(IN2L, HIGH);
            }
             if(controller->axisRY() < 0) { // negative y is downward on stick
                Serial.println(" DC motor move backward");
                digitalWrite(IN1R, HIGH);
                digitalWrite(IN2R, LOW);
                digitalWrite(IN1L, HIGH);
                digitalWrite(IN2L, LOW);
            }
            if(controller->axisRX() > 0) { // negative y is upward on stick
                Serial.println(" DC motor rotate clockwise");
                digitalWrite(IN1R, HIGH);
                digitalWrite(IN2R, LOW);
                digitalWrite(IN1L, LOW);
                digitalWrite(IN2L, HIGH);
            }
             if(controller->axisRX() < 0) { // negative y is downward on stick
                Serial.println(" DC motor rotate counterclockwise");
                digitalWrite(IN1R, LOW);
                digitalWrite(IN2R, HIGH);
                digitalWrite(IN1L, HIGH);
                digitalWrite(IN2L, LOW);
            }
            if(controller->axisRY() == 0 && controller->axisRX() == 0) { // stop motor 1
                Serial.println(" DC motor stop");
                digitalWrite(IN1R, LOW);
                digitalWrite(IN2R, LOW);
                digitalWrite(IN1L, LOW);
                digitalWrite(IN2L, LOW);
            }
            

            // PHYSICAL BUTTON A
            if (controller->b()) {
                Serial.println("button a pressed");
            }

        }
        vTaskDelay(1);
    }
    
}           
