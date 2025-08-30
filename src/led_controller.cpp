
#pragma once // prevents multiple inclusion
#include <led_controller.h>
#include <Arduino.h>

// RGB LED pins
const int redPin   = 27;
const int greenPin = 26;
const int bluePin  = 25;

// PWM channel assignment (ESP32 has 16 channels: 0-15)
const int redChannel   = 0;
const int greenChannel = 1;
const int blueChannel  = 2;

// PWM frequency and resolution
const int freq = 5000;     // 5 KHz
const int resolution = 8;  // 8-bit (0-255)

void setLeds(){
    // Configure LEDC channels
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);

  // Attach channels to pins
  ledcAttachPin(redPin, redChannel);
  ledcAttachPin(greenPin, greenChannel);
  ledcAttachPin(bluePin, blueChannel);

}

void setColorRGB(int red, int green, int blue, float brightness) {
    // Clamp brightness between 0 and 1
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;

    // Scale each channel by brightness
    int r = (int)(red * brightness);
    int g = (int)(green * brightness);
    int b = (int)(blue * brightness);

    // Write to LED channels
    ledcWrite(redChannel, r);
    ledcWrite(greenChannel, g);
    ledcWrite(blueChannel, b);
}
