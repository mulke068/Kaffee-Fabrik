// Settings for ESP32 Controller

// #pragma once
#ifndef SETTINGS_H
#define SETTINGS_H


#include <Arduino.h> // TODO: remove this dependency

#define SDA_PIN 17
#define SCL_PIN 18
#define PCA9555_ADDR 0x20
#define INA219_ADDR 0x40
#define TMP102_IC20_ADDR 0x48
#define TMP102_IC20_ALERT 1
#define TMP102_IC8_ADDR 0x49
#define TMP102_IC8_ALERT 2

const uint8_t PWMA_1 = 4;
const uint8_t PWMB_1 = 5;
const uint8_t PWMA_2 = 6;
const uint8_t PWMB_2 = 7;

const uint8_t AIN1_1 = 16;
const uint8_t AIN2_1 = 15;
const uint8_t BIN1_1 = 8;
const uint8_t BIN2_1 = 9;

const uint8_t AIN1_2 = 11;
const uint8_t AIN2_2 = 10;
const uint8_t BIN1_2 = 13;
const uint8_t BIN2_2 = 14;

const uint8_t STBY = 19;
// const uint8_t BUTTON_PIN = 0;
#define BUTTON_PIN 0

// Globals for sensors and LED state
// Adafruit_INA219 ina219(INA219_ADDR);
// const byte port0State = 0b00000000;
// const byte port1State = 0b00000000;

// LED PCA9555
const byte led1 = 0b00000001;
const byte led2 = 0b00000010;
const byte led3 = 0b00000100;
const byte led4 = 0b00001000;
const byte led5 = 0b00010000;
const byte led6 = 0b00100000;
const byte led7 = 0b00100000; // Port 1 mapping (LEDs 7-12)
const byte led8 = 0b00010000;
const byte led9 = 0b00001000;
const byte led10 = 0b00000100;
const byte led11 = 0b00000010;
const byte led12 = 0b00000001;
const uint8_t led17 = 48;
const uint8_t led18 = 47;


#endif