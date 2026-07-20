/**
 * @file ADS8685.cpp
 * @brief TI ADS8685 16-bit SAR ADC 驱动实现
 * @date 2026-07-17
 */

#include "ADS8685.h"

/* 命令帧头 (表 7-5): WRITE=0xD0, READ_HWORD=0xC8 */
#define CMD_WRITE_BYTE0     0xD0
#define CMD_READHWORD_BYTE0 0xC8

/* RST_PWRCTL_REG 写保护: 先写地址 0x05 = 0x69 解锁 */
#define RST_UNLOCK_ADDR 0x05
#define RST_UNLOCK_KEY  0x69

/* 量程倍数查表 (索引 = RANGE_SEL[3:0]), VREF 的系数 */
static const float RANGE_MULT[16] = {
    3.0f, 2.5f, 1.5f, 1.25f, 0.625f, 0, 0, 0,   /* 0x0~0x4 双极性 */
    3.0f, 2.5f, 1.5f, 1.25f, 0, 0, 0, 0          /* 0x8~0xB 单极性 */
};

ADS8685::ADS8685(uint8_t csPin, SPIClass &spi, uint32_t spiFreq,
                 int8_t rstPin, float vref)
    : _spi(spi), _spiSettings(spiFreq, MSBFIRST, SPI_MODE0),
      _csPin(csPin), _rstPin(rstPin), _vref(vref),
      _range(ADS8685_RANGE_BIPOLAR_3VREF) {
}

bool ADS8685::begin(bool initSpiBus) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, HIGH);
    }

    if (initSpiBus) _spi.begin();

    /* 填充流水线: 32位完整空读 + 启动首次转换
     * 注意: 必须用完整32 SCLK帧, 短帧(<32)会被芯片当作NOP */
    _spi.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
    _spi.transfer16(0x0000);
    _spi.transfer16(0x0000);
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
    delayMicroseconds(CONV_WAIT_US);
    return true;
}

void ADS8685::hardwareReset() {
    if (_rstPin < 0) return;
    digitalWrite(_rstPin, LOW);
    delayMicroseconds(1);
    digitalWrite(_rstPin, HIGH);
    delay(20);
    begin(false);
}

/* ================ 数据读取 ================ */
uint16_t ADS8685::readRaw() {
    _spi.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
    /* 完整 32 位读取: D[31:16]=转换结果, D[15:0]=标志(丢弃) */
    uint16_t code = _spi.transfer16(0x0000);
    _spi.transfer16(0x0000);
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
    delayMicroseconds(CONV_WAIT_US);
    return code;
}

float ADS8685::readVoltage() {
    return codeToVoltage(readRaw());
}

/* ================ 寄存器访问 ================ */
uint16_t ADS8685::readRegister(uint8_t addr) {
    _spi.beginTransaction(_spiSettings);

    /* 帧 F: 发送 READ_HWORD 命令 (32 SCLK) */
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
    _spi.transfer(CMD_READHWORD_BYTE0);
    _spi.transfer(addr);
    _spi.transfer16(0x0000);
    digitalWrite(_csPin, HIGH);
    delayMicroseconds(CONV_WAIT_US);

    /* 帧 F+1: 读取 16-bit 寄存器数据 (32 SCLK) */
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
    uint16_t value = _spi.transfer16(0x0000);
    _spi.transfer16(0x0000);
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
    delayMicroseconds(CONV_WAIT_US);
    return value;
}

void ADS8685::writeRegister(uint8_t addr, uint16_t value) {
    /* RST_PWRCTL_REG 需先解锁 */
    if (addr == ADS8685_REG_RST_PWRCTL) {
        _spi.beginTransaction(_spiSettings);
        digitalWrite(_csPin, LOW);
        delayMicroseconds(1);
        _spi.transfer(CMD_WRITE_BYTE0);
        _spi.transfer(RST_UNLOCK_ADDR);
        _spi.transfer16(RST_UNLOCK_KEY);
        digitalWrite(_csPin, HIGH);
        _spi.endTransaction();
        delayMicroseconds(CONV_WAIT_US);
    }

    _spi.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    delayMicroseconds(1);
    _spi.transfer(CMD_WRITE_BYTE0);
    _spi.transfer(addr);
    _spi.transfer16(value);
    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
    delayMicroseconds(CONV_WAIT_US);
}

/* ================ 配置 ================ */
void ADS8685::setRange(ADS8685_Range range) {
    /* 1. 尝试写入量程 */
    writeRegister(ADS8685_REG_RANGE_SEL, (uint16_t)(range & 0x0F));

    /* 2. 回读芯片实际生效的量程, 用真实值换算保证电压正确。
     *    若写入失败(SDI异常), 芯片保持默认±3VREF(code=0x0)。 */
    uint8_t code = readRegister(ADS8685_REG_RANGE_SEL) & 0x0F;
    switch (code) {
        case 0x0: _range = ADS8685_RANGE_BIPOLAR_3VREF;     break;
        case 0x1: _range = ADS8685_RANGE_BIPOLAR_2_5VREF;   break;
        case 0x2: _range = ADS8685_RANGE_BIPOLAR_1_5VREF;   break;
        case 0x3: _range = ADS8685_RANGE_BIPOLAR_1_25VREF;  break;
        case 0x4: _range = ADS8685_RANGE_BIPOLAR_0_625VREF; break;
        case 0x8: _range = ADS8685_RANGE_UNIPOLAR_3VREF;    break;
        case 0x9: _range = ADS8685_RANGE_UNIPOLAR_2_5VREF;  break;
        case 0xA: _range = ADS8685_RANGE_UNIPOLAR_1_5VREF;  break;
        case 0xB: _range = ADS8685_RANGE_UNIPOLAR_1_25VREF; break;
        default:  _range = ADS8685_RANGE_BIPOLAR_3VREF;     break;
    }

    /* 3. 丢弃旧量程结果 (用完整32位帧) */
    readRaw();
}

void ADS8685::enableInternalReference(bool enable) {
    uint16_t reg = readRegister(ADS8685_REG_RANGE_SEL);
    if (enable) reg &= ~(1 << 6);
    else        reg |=  (1 << 6);
    writeRegister(ADS8685_REG_RANGE_SEL, reg);
}

/* ================ 换算工具 ================ */
void ADS8685::getRangeLimits(float &vMin, float &vMax) const {
    uint8_t idx = (uint8_t)_range & 0x0F;
    float mult = RANGE_MULT[idx];
    if (idx >= 0x8) {        /* 单极性 */
        vMin = 0.0f;
        vMax = mult * _vref;
    } else {                 /* 双极性 */
        vMin = -mult * _vref;
        vMax =  mult * _vref;
    }
}

float ADS8685::getLSB() const {
    float vMin, vMax;
    getRangeLimits(vMin, vMax);
    return (vMax - vMin) / 65536.0f;
}

float ADS8685::codeToVoltage(uint16_t code) const {
    float vMin, vMax;
    getRangeLimits(vMin, vMax);
    return vMin + (float)code * (vMax - vMin) / 65536.0f;
}