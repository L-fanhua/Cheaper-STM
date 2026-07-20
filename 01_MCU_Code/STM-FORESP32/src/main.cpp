/**
 * @file main.cpp
 * @brief ADS8685 16-bit ADC 读取示例
 * @date 2026-07-17
 *
 * 接线:
 *   GPIO9  (CS)   -> ADC CONVST/CS
 *   GPIO10 (SCLK) -> ADC SCLK
 *   GPIO11 (MOSI) -> ADC SDI
 *   GPIO12 (MISO) -> ADC SDO-0
 */

#include <Arduino.h>
#include <SPI.h>
#include <ADS8685.h>

#define PIN_CS    9
#define PIN_SCLK  10
#define PIN_SDI   11
#define PIN_SDO   12

SPIClass adcSPI(FSPI);
ADS8685 adc(PIN_CS, adcSPI, 4000000UL);

void setup() {
    Serial.begin(115200);
    delay(500);

    adcSPI.begin(PIN_SCLK, PIN_SDO, PIN_SDI, -1);
    adc.begin();
    adc.setRange(ADS8685_RANGE_BIPOLAR_2_5VREF);   /* 请求 ±10.24V */

    Serial.println();
    Serial.println(F("== ADS8685 ADC =="));

    /* 显示芯片实际生效的量程 (自适应回读) */
    uint16_t rangeReg = adc.readRegister(ADS8685_REG_RANGE_SEL);
    Serial.print(F("RANGE_SEL reg = 0x"));
    Serial.println(rangeReg, HEX);

    float vMin, vMax;
    adc.getRangeLimits(vMin, vMax);
    Serial.print(F("Active range: "));
    Serial.print(vMin, 2);
    Serial.print(F(" ~ "));
    Serial.print(vMax, 2);
    Serial.println(F(" V"));
    Serial.println(F("Voltage(V)\tRaw"));
}

void loop() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        uint16_t raw = adc.readRaw();
        float v = adc.codeToVoltage(raw);
        Serial.print(v, 5);
        Serial.print(F("\t\t"));
        Serial.println(raw);
    }
    delay(1);
}