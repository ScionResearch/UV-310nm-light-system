#include "defines.h"

// Forward declarations
void initPWM();
int16_t floatToUint16(float value, float multiplier);
bool temperatureControl();
void updateLedPower();
void killLEDs();
void packModbusData();
void extractModbusData();

uint32_t timestamp = 0;

void setup() {   
    asm(".global _printf_float"); 
    // Pin configurations
    pinMode(PIN_ENABLE, INPUT_PULLUP);
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
    digitalWrite(PIN_LED_3, HIGH);
    pinMode(PIN_FAN_1, OUTPUT);
    pinMode(PIN_FAN_2, OUTPUT);
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    analogReadResolution(12); // 12-bit ADC resolution
    analogWriteResolution(8); // 8-bit PWM resolution

    // Initialise PWM to 100Hz
    //initPWM();

    // Debug Serial
    Serial.begin(115200);
    if (debug) {
        while (!Serial) {
            delay(10);
        }
        Serial.println("Starting UV-310nm Light System...");
    }

    // Initialize NTC thermistors
    ntc1.begin();
    ntc2.begin();
    ntc3.begin();

    if (ntc1.isConnected() && ntc2.isConnected() && ntc3.isConnected()) {
        Serial.println("All NTC thermistors connected.");
    } else {
        Serial.println("Warning: One or more NTC thermistors not connected!");
        thermistorsOK = false;
    }

    Serial.printf("NTC1: %.2f C, NTC2: %.2f C, NTC3: %.2f C\n", ntc1.temperature(), ntc2.temperature(), ntc3.temperature());

    modbus.configureHoldingRegisters(holdingRegisters, 20);
    modbus.configureInputRegisters(inputRegisters, 20);
    modbus.begin(1, 115200, SERIAL_8N1);

    // Populate registers with initial values
    inputData.ntcTemp[0] = floatToUint16(ntc1.temperature(), 0.1);
    inputData.ntcTemp[1] = floatToUint16(ntc2.temperature(), 0.1);
    inputData.ntcTemp[2] = floatToUint16(ntc3.temperature(), 0.1);
    inputData.fanSpeed = 0;
    holdingData.ledPower[0] = 100;
    holdingData.ledPower[1] = 100;
    holdingData.ledPower[2] = 100;

    packModbusData();
    memcpy(holdingRegisters, &holdingData, sizeof(holdingData));

    Serial.println("Ready.");
    digitalWrite(LED_BUILTIN, HIGH);

    timestamp = millis();
}

void loop() {
    if(digitalRead(PIN_ENABLE)) killLEDs();

    packModbusData();
    int functionCode = modbus.poll();
    if (functionCode == 0x06 || functionCode == 0x10) {
        if (!digitalRead(PIN_ENABLE)) updateLedPower();
        else Serial.println("Ignoring LED power update, driver disabled!");
    } else if (functionCode == 0x04) {
        Serial.println("Modbus Read Input Registers");
    }

    

    if (millis() - timestamp >= 1000) {
        timestamp = millis();
        if (!temperatureControl()) {
            Serial.println("Thermistor error detected!");
            digitalWrite(LED_BUILTIN, LOW);
        } else {
            Serial.printf("T1: %.2f C, T2: %.2f C, T3: %.2f C | Fans: %.1f %%\n", 
                 ntc1.temperature(), ntc2.temperature(), ntc3.temperature(), inputData.fanSpeed * 0.1);
            digitalWrite(LED_BUILTIN, HIGH);
        }
    }
}

// Utility function to convert float to int16_t with scaling
int16_t floatToUint16(float value, float multiplier) {
    if (isnan(value)) {
        thermistorsOK = false;
        return 0;
    }
    value /= multiplier;
    return round(value);
}

bool temperatureControl() {
    // Get temperatures
    float temp1 = ntc1.temperature();
    float temp2 = ntc2.temperature();    
    float temp3 = ntc3.temperature();
    if (isnan(temp1) || isnan(temp2) || isnan(temp3)) {
        thermistorsOK = false;
        return false;
    }

    inputData.ntcTemp[0] = floatToUint16(temp1, 0.1);
    inputData.ntcTemp[1] = floatToUint16(temp2, 0.1);
    inputData.ntcTemp[2] = floatToUint16(temp3, 0.1);

    if (abs(temp1 - temp2) > 5.0 || abs(temp1 - temp3) > 5.0 || abs(temp2 - temp3) > 5.0) {
        thermistorsOK = false;
        return false;
    }

    thermistorsOK = true;
    float tempAggregated = (temp1 + temp2 + temp3) / 3.0;
    float error = tempAggregated - holdingData.tempSetpoint;

    float fanSpeed = (error * 20);
    if (fanSpeed < 20.0) fanSpeed = 20.0;
    if (fanSpeed > 100.0) fanSpeed = 100.0;
    inputData.fanSpeed = floatToUint16(fanSpeed, 0.1);

    uint8_t pwmValue = (uint8_t)(fanSpeed * 2.55);
    analogWrite(PIN_FAN_1, pwmValue);
    analogWrite(PIN_FAN_2, pwmValue);

    Serial.printf("T: %.2f C, E: %.2f C, fanSpeed: %.1f %%, PWM: %d\n", tempAggregated, error, fanSpeed, pwmValue);

    return true;
}

void updateLedPower() {
    extractModbusData();
    if (!thermistorsOK) {
        killLEDs();
        return;
    }

    uint8_t pwm1 = constrain((uint8_t)(holdingData.ledPower[0] * 0.255), 0, 255);
    uint8_t pwm2 = constrain((uint8_t)(holdingData.ledPower[1] * 0.255), 0, 255);
    uint8_t pwm3 = constrain((uint8_t)(holdingData.ledPower[2] * 0.255), 0, 255);

    // Handle power below min PWM by switching some banks off
    if (pwm1 == pwm2 && pwm1 == pwm3 && pwm1 < LED_MIN_PWM) {
        uint8_t pwmTot = pwm1 + pwm2 + pwm3;
        if (pwmTot > LED_MIN_PWM * 2) {
            uint8_t pwmAdj = pwmTot/2;
            pwm1 = pwmAdj;
            pwm2 = 0;
            pwm3 = pwmAdj;
        } else if (pwmTot > LED_MIN_PWM) {
            pwm1 = 0;
            pwm2 = pwmTot;
            pwm3 = 0;
        }
    }

    Serial.printf("Updated LED PWM Values: %d, %d, %d\n", pwm1, pwm2, pwm3);

    analogWrite(PIN_LED_1, 255 - pwm1);
    analogWrite(PIN_LED_2, 255 - pwm2);
    analogWrite(PIN_LED_3, 255 - pwm3);
}

void killLEDs() {
    analogWrite(PIN_LED_1, 255);
    analogWrite(PIN_LED_2, 255);
    analogWrite(PIN_LED_3, 255);
}

void packModbusData() {
    memcpy(inputRegisters, &inputData, sizeof(inputData));
}

void extractModbusData() {
    memcpy(&holdingData, holdingRegisters, sizeof(holdingData));
}