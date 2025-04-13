#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <stdint.h>

namespace Hardware
{
    constexpr uint8_t SDA = 17;
    constexpr uint8_t SCL = 18;
    constexpr uint8_t STBY = 19;
    constexpr uint8_t BUTTON_PIN = 0;

    namespace Motor1
    {
        constexpr uint8_t PWMA = 4;
        constexpr uint8_t AIN1 = 16;
        constexpr uint8_t AIN2 = 15;
    }
    namespace Motor2
    {
        constexpr uint8_t PWMB = 5;
        constexpr uint8_t BIN1 = 8;
        constexpr uint8_t BIN2 = 9;
    }
    namespace Motor3
    {
        constexpr uint8_t PWMA = 6;
        constexpr uint8_t AIN1 = 11;
        constexpr uint8_t AIN2 = 10;
    }
    namespace Motor4
    {
        constexpr uint8_t PWMB = 7;
        constexpr uint8_t BIN1 = 13;
        constexpr uint8_t BIN2 = 14;
    }
}

namespace SensorAdress
{
    constexpr uint8_t PCA9555 = 0x20;
    constexpr uint8_t INA219 = 0x40;
    constexpr uint8_t TMP102_IC20 = 0x48;
    constexpr uint8_t TMP102_IC8 = 0x49;

    namespace Alert
    {
        constexpr uint8_t TMP102_IC20 = 1;
        constexpr uint8_t TMP102_IC8 = 2;

    }
}

// LED PCA9555
namespace LedAdress
{
    constexpr uint8_t led1 = 0b00000001;
    constexpr uint8_t led2 = 0b00000010;
    constexpr uint8_t led3 = 0b00000100;
    constexpr uint8_t led4 = 0b00001000;
    constexpr uint8_t led5 = 0b00010000;
    constexpr uint8_t led6 = 0b00100000;
    constexpr uint8_t led7 = 0b00100000; // Port 1 mapping (LEDs 7-12)
    constexpr uint8_t led8 = 0b00010000;
    constexpr uint8_t led9 = 0b00001000;
    constexpr uint8_t led10 = 0b00000100;
    constexpr uint8_t led11 = 0b00000010;
    constexpr uint8_t led12 = 0b00000001;
    constexpr uint8_t led17 = 48;
    constexpr uint8_t led18 = 47;
}

#endif