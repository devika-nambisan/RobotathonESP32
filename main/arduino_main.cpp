// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>
#include <Arduino_APDS9960.h>

#define LED 2
#define IN1R 12
#define IN2R 14
#define IN1L 13
#define IN2L 15

#define APDS9960_INT 0 
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

Servo servo;

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

void setup() {
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
 
    servo.attach(16);

    // motor controller outputs
    pinMode(IN1R, OUTPUT);
    pinMode(IN2R, OUTPUT);
    pinMode(IN1L, OUTPUT);
    pinMode(IN2L, OUTPUT);
    pinMode(LED, OUTPUT);

    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();

    Serial.begin(115200);
}

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
            
            while(!sensor.colorAvailable()) {
                delay(5);
            }
    
            int r, g, b, a;
            sensor.readColor(r, g, b, a);

            Serial.print("r = ");
            Serial.print(r);
            Serial.print("g = ");
            Serial.print(g);
            Serial.print("b = ");
            Serial.println(b);

            // PHYSICAL BUTTON A
            if (controller->b()) {
                Serial.println("Color sensing");
                if (r - g > 40 && r - b > 40) {
                    Serial.println("Red");
                    digitalWrite(LED, HIGH);
                    delay(2000);
                    digitalWrite(LED, LOW);
                }
                else if (b - r > 40 && b - g > 40) {
                    Serial.println("Blue");
                    digitalWrite(LED, HIGH);
                    delay(1000);
                    digitalWrite(LED, LOW);
                    delay(250);
                    digitalWrite(LED, HIGH);
                    delay(1000);
                    digitalWrite(LED, LOW);
                }
                else if (g - r > 40 && g - b > 40) {
                    Serial.println("Green");
                    digitalWrite(LED, HIGH);
                    delay(500);
                    digitalWrite(LED, LOW);
                    delay(250);
                    digitalWrite(LED, HIGH);
                    delay(500);
                    digitalWrite(LED, LOW);
                    delay(250);
                    digitalWrite(LED, HIGH);
                    delay(500);
                    digitalWrite(LED, LOW);
                }
            }
        }
    }
    
    vTaskDelay(1);
}           