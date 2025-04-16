/**
 * @file LedController.h
 * @author Kevin Muller (@kevbcef.com)
 * @brief Header file for the LedController class.
 * This class manages the LED states and patterns for the ESP32 board.
 * It provides methods to set individual LEDs, toggle their states, and control all LEDs at once.
 * @version 1.0
 * @date 2025-04-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef LedController_h
#define LedController_h

#include <HardwareSerial.h>
#include <Wire.h>
#include "HardwareConfig.h"
#include "GlobalConfig.h"

class LedController
{
public:
    LedController();
    ~LedController();
    void begin();

    void setLeds(uint8_t port0, uint8_t port1);
    void runLedPattern(int patternNum);

    void setLed(int ledNum, bool state);
    void toggleLed(int ledNum);

    void setAllOn();
    void setAllOff();

    uint8_t getPort0State() const { return _port0State; };
    uint8_t getPort1State() const { return _port1State; };

private:
    bool getLedMaskAndPort(int ledNum, uint8_t &ledmask, uint8_t *&portState);

    uint8_t _port0State;
    uint8_t _port1State;
};

#endif // LedController_h