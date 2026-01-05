#pragma once

#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <NTC_Therm.h>

bool debug = false;

// Pins
#define PIN_ENABLE 0
#define PIN_NTC_1 3
#define PIN_NTC_2 4
#define PIN_NTC_3 5
#define PIN_FAN_1 1
#define PIN_FAN_2 2
#define PIN_TX 6
#define PIN_RX 7
#define PIN_LED_1 10
#define PIN_LED_2 9
#define PIN_LED_3 8

// Constants
#define LED_MIN_PWM 20

// Library objects
ModbusRTUSlave modbus(Serial1);
NTC_Therm ntc1(PIN_NTC_1, 10000.0, 10000.0, 3950.0, true, 12, 3.3);
NTC_Therm ntc2(PIN_NTC_2, 10000.0, 10000.0, 3950.0, true, 12, 3.3);
NTC_Therm ntc3(PIN_NTC_3, 10000.0, 10000.0, 3950.0, true, 12, 3.3);

// Modbus data structure
struct ModbusHoldingData {
    uint16_t ledPower[3]; // LED power levels in 0.1 %
    int16_t tempSetpoint = 30;   // Temperature setpoint for fan control in degrees C
};

struct ModbusInputData {
    int16_t ntcTemp[3]; // NTC temperatures in 0.1 degrees C
    uint16_t fanSpeed; // Fan speed in 0.1 %    
};

ModbusHoldingData holdingData;
ModbusInputData inputData;
uint16_t holdingRegisters[20] = {0};
uint16_t inputRegisters[20] = {0};

bool thermistorsOK = false;
bool ledsEnabled[3] = {false, false, false};