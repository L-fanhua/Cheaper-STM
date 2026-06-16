/**
 * @file main.cpp
 * @brief AD5761RBRUZ-RL7 多片 DAC 控制示例(4 芯片,共用 SPI 总线)
 * @date 2026-06-16
 *
 * 硬件连接 (ESP32-S3-DevKitM-1):
 *   ESP32-S3 GPIO     ->  AD5761 × 4
 *   -------------------------------------------
 *   GPIO12 (SCK)      ->  SCLK  (4 片共用, ESP32-S3 默认 SPI SCK)
 *   GPIO11 (MOSI)     ->  SDIN  (4 片共用, ESP32-S3 默认 SPI MOSI)
 *   GPIO13 (MISO)     ->  SDO   (4 片共用, 可选)
 *   GPIO5  (SYNC1)    ->  /SYNC (第 1 片片选)
 *   GPIO6  (SYNC2)    ->  /SYNC (第 2 片片选)
 *   GPIO7  (SYNC3)    ->  /SYNC (第 3 片片选)
 *   GPIO8  (SYNC4)    ->  /SYNC (第 4 片片选)
 *   GPIO9  (LDAC)     ->  /LDAC (4 片共用, 实现同步更新)
 *   GPIO14 (RESET)    ->  /RESET(4 片共用, 可选)
 *
 *   注意:AD5761 为双电源器件,需 ±12V~±15V 双电源及 +3.3V 逻辑供电,
 *   初次调试时务必确认电源时序与基准配置后再接入负载。
 */

#include <Arduino.h>
#include <AD5761.h>

/* ----------------------- 引脚定义 ----------------------- */
static const uint8_t SYNC_PINS[4] = {5, 6, 7, 8};  // SYNC1~SYNC4
static const uint8_t LDAC_PIN     = 9;              // 共用 LDAC
static const int8_t  RESET_PIN    = 14;             // 共用 RESET(-1 不使用)

/* 共用 SPI (默认 SPI: SCK=GPIO12, MOSI=GPIO11, MISO=GPIO13) */
AD5761Array dacs(SYNC_PINS, 4, SPI, 1000000UL, LDAC_PIN, /*clr=*/-1, RESET_PIN);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println(F("== AD5761 x 4 DAC Array Example =="));

    /* 1. 初始化所有芯片(内部仅初始化一次 SPI 总线) */
    dacs.begin();
    delay(10);

    /* 2. 统一配置输出范围:±10V 双极性,外部基准 */
    dacs.setOutputRangeAll(RANGE_BIPOLAR_10V, false);
    delay(10);

    /* 3. 演示一: 4 路同步输出 0V
     *    先准备数据到各芯片输入寄存器,再通过共用 LDAC 同步更新 */
    Serial.println(F("[Demo 1] Synchronous 0V on all channels"));
    dacs.prepareVoltageAll(0.0f);
    dacs.latchAll();     // <-- 同一时刻 4 路全部更新
    delay(1000);

    /* 4. 演示二: 4 路输出不同电压(独立设置,立即生效) */
    Serial.println(F("[Demo 2] Independent voltages: +5, +2.5, -2.5, -5 V"));
    float voltages[4] = { 5.0f, 2.5f, -2.5f, -5.0f };
    dacs.setVoltages(voltages);
    delay(2000);

    /* 5. 演示三: 仅修改第 3 路(索引 2) */
    Serial.println(F("[Demo 3] Set channel 3 to +1.234 V"));
    dacs.setVoltage(2, 1.234f);
    delay(1000);

    Serial.println(F("AD5761 array init done."));
}

void loop() {
    /* 周期性扫描:4 路同步输出正弦波 (-5V ~ +5V)
     * 使用 prepareVoltages + latchAll 实现"先准备后同步更新" */
    static float phase = 0.0f;

    float v = sinf(phase) * 5.0f;            // -5V ~ +5V 正弦
    float voltages[4] = { v, -v, v * 0.5f, -v * 0.5f };

    dacs.prepareVoltages(voltages);
    dacs.latchAll();                          // 4 路同步更新

    phase += 0.05f;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;

    delay(20);                                // 约 50Hz
}