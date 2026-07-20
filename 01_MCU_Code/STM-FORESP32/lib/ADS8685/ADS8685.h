/**
 * @file ADS8685.h
 * @brief TI ADS8681/ADS8685/ADS8689 16-bit 单通道 SAR ADC 驱动
 * @date 2026-07-17
 *
 * 16-bit 单通道 SAR ADC, 内置 PGA/滤波器/4.096V 基准, 单 5V 供电。
 * SPI_MODE0 接口 (SPI-00-S), CONVST/CS 双功能引脚。
 *
 * 引脚: SDI, SDO-0, SCLK, CONVST/CS (SDO-1 为可选双输出, 本库不用)
 */

#ifndef ADS8685_H
#define ADS8685_H

#include <Arduino.h>
#include <SPI.h>

/* 寄存器地址 (数据手册表 7-10) */
#define ADS8685_REG_DEVICE_ID     0x00
#define ADS8685_REG_RST_PWRCTL   0x04
#define ADS8685_REG_SDI_CTL      0x08
#define ADS8685_REG_SDO_CTL      0x0C
#define ADS8685_REG_DATAOUT_CTL  0x10
#define ADS8685_REG_RANGE_SEL    0x14
#define ADS8685_REG_ALARM        0x20
#define ADS8685_REG_ALARM_H_TH   0x24
#define ADS8685_REG_ALARM_L_TH   0x28

/* 输入量程 (RANGE_SEL[3:0], VREF=4.096V) */
enum ADS8685_Range {
    ADS8685_RANGE_BIPOLAR_3VREF     = 0x0,  ///< ±12.288V (默认)
    ADS8685_RANGE_BIPOLAR_2_5VREF   = 0x1,  ///< ±10.24V
    ADS8685_RANGE_BIPOLAR_1_5VREF   = 0x2,  ///< ±6.144V
    ADS8685_RANGE_BIPOLAR_1_25VREF  = 0x3,  ///< ±5.12V
    ADS8685_RANGE_BIPOLAR_0_625VREF = 0x4,  ///< ±2.56V
    ADS8685_RANGE_UNIPOLAR_3VREF    = 0x8,  ///< 0~12.288V
    ADS8685_RANGE_UNIPOLAR_2_5VREF  = 0x9,  ///< 0~10.24V
    ADS8685_RANGE_UNIPOLAR_1_5VREF  = 0xA,  ///< 0~6.144V
    ADS8685_RANGE_UNIPOLAR_1_25VREF = 0xB,  ///< 0~5.12V
};

/**
 * @class ADS8685
 * @brief ADS8685 单通道 16-bit SAR ADC 驱动
 *
 * @code
 *   SPIClass spi(FSPI);
 *   ADS8685 adc(9, spi, 4000000UL);
 *   void setup() {
 *       spi.begin(10, 12, 11, -1);   // SCK, MISO, MOSI
 *       adc.begin();
 *       adc.setRange(ADS8685_RANGE_BIPOLAR_2_5VREF);
 *   }
 *   void loop() {
 *       float v = adc.readVoltage();
 *   }
 * @endcode
 */
class ADS8685 {
public:
    ADS8685(uint8_t csPin, SPIClass &spi = SPI,
            uint32_t spiFreq = 4000000UL,
            int8_t rstPin = -1, float vref = 4.096f);

    /** @brief 初始化。initSpiBus=false 时不在内部调用 spi.begin() */
    bool begin(bool initSpiBus = false);

    void hardwareReset();

    /* 数据读取 */
    uint16_t readRaw();
    float    readVoltage();

    /* 寄存器访问 */
    uint16_t readRegister(uint8_t addr);
    void     writeRegister(uint8_t addr, uint16_t value);

    /* 配置 */
    void setRange(ADS8685_Range range);
    void enableInternalReference(bool enable);

    /* 换算工具 */
    float codeToVoltage(uint16_t code) const;
    void  getRangeLimits(float &vMin, float &vMax) const;
    float getLSB() const;

    ADS8685_Range getRange() const { return _range; }
    float getVRef() const { return _vref; }
    void  setVRef(float vref) { _vref = vref; }

private:
    SPIClass     &_spi;
    SPISettings   _spiSettings;
    uint8_t       _csPin;
    int8_t        _rstPin;
    float         _vref;
    ADS8685_Range _range;

    static constexpr uint16_t CONV_WAIT_US = 5;  ///< tconv+tacq
};

#endif /* ADS8685_H */