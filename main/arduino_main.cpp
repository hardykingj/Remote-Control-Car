/****************************************************************************
http://retro.moe/unijoysticle2

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
#endif // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>

static GamepadPtr myGamepad;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    // In this example we only use one gamepad at the same time.
    myGamepad = gp;
    Serial.println("CALLBACK: Gamepad is connected!");
}

void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("CALLBACK: Gamepad is disconnected!");
    myGamepad = nullptr;
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    String fv = BP32.firmwareVersion();
    Serial.print("Firmware: ");
    Serial.println(fv);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    if (myGamepad && myGamepad->isConnected()) {
        // There are different ways to query whether a button is pressed.
        // By query each button individually:
        //  a(), b(), x(), y(), l1(), etc...
        if (myGamepad->a()) {
            static int colorIdx = 0;
            // Some gamepads like DS4 and DualSense support changing the color LED.
            // It is possible to change it by calling:
            switch (colorIdx % 3) {
                case 0:
                    // Red
                    myGamepad->setColorLED(255, 0, 0);
                    break;
                case 1:
                    // Green
                    myGamepad->setColorLED(0, 255, 0);
                    break;
                case 2:
                    // Blue
                    myGamepad->setColorLED(0, 0, 255);
                    break;
            }
            colorIdx++;
        }

        if (myGamepad->b()) {
            // Turn on the 4 LED. Each bit represents one LED.
            static int led = 0;
            led++;
            // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
            // support changing the "Player LEDs": those 4 LEDs that usually indicate
            // the "gamepad seat".
            // It is possible to change them by calling:
            myGamepad->setPlayerLEDs(led & 0x0f);
        }

        if (myGamepad->x()) {
            // Duration: 255 is ~2 seconds
            // force: intensity
            // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
            // rumble.
            // It is possible to set it by calling:
            myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
        }

        // Another way to query the buttons, is by calling buttons(), or
        // miscButtons() which return a bitmask.
        // Some gamepads also have DPAD, axis and more.
        char buffer[120];
        snprintf(buffer, sizeof(buffer) - 1,
                 "dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, "
                 "%4d, brake: %4d, throttle: %4d, misc: 0x%02x",
                 myGamepad->dpad(),        // DPAD
                 myGamepad->buttons(),     // bitmask of pressed buttons
                 myGamepad->axisX(),       // (-511 - 512) left X Axis
                 myGamepad->axisY(),       // (-511 - 512) left Y axis
                 myGamepad->axisRX(),      // (-511 - 512) right X axis
                 myGamepad->axisRY(),      // (-511 - 512) right Y axis
                 myGamepad->brake(),       // (0 - 1023): brake button
                 myGamepad->throttle(),    // (0 - 1023): throttle (AKA gas) button
                 myGamepad->miscButtons()  // bitmak of pressed "misc" buttons
        );
        Serial.println(buffer);

        // You can query the axis and other properties as well. See Gamepad.h
        // For all the available functions.
    }

    delay(150);
}
