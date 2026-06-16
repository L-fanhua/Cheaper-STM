/**
 * @file AD5761.cpp
 * @brief AD5761RBRUZ-RL7 16-bit 单通道双极性 DAC 驱动实现
 *        含单芯片 AD5761 与多芯片 AD5761Array 实现
 * @date 2026-06-16
 */

#include "AD5761.h"

/* ================================================================== *
 *  单芯片驱动 AD5761 实现
 * ================================================================== */
AD5761::AD5761(uint8_t csPin,
               SPIClass &spi,
               uint32_t spiFreq,
               int8_t ldacPin,
               int8_t clrPin,
               int8_t resetPin)
    : _spi(spi),
      _spiSettings(spiFreq, MSBFIRST, SPI_MODE1),
      _csPin(csPin),
      _ldacPin(ldacPin),
      _clrPin(clrPin),
      _resetPin(resetPin),
      _range(RANGE_BIPOLAR_10V),
      _useIntRef(false) {
}

bool AD5761::begin(bool initSpiBus) {
    /* 配置片选引脚 (默认拉高,空闲状态) */
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    /* 配置 LDAC 引脚:默认拉低 = 透明模式(写入即更新) */
    if (_ldacPin >= 0) {
        pinMode(_ldacPin, OUTPUT);
        digitalWrite(_ldacPin, LOW);
    }

    /* 配置 CLR 引脚:默认拉高(不清零) */
    if (_clrPin >= 0) {
        pinMode(_clrPin, OUTPUT);
        digitalWrite(_clrPin, HIGH);
    }

    /* 配置 RESET 引脚:默认拉高(不复位) */
    if (_resetPin >= 0) {
        pinMode(_resetPin, OUTPUT);
        digitalWrite(_resetPin, HIGH);
    }

    /* 仅在请求时初始化 SPI 总线(多芯片共享总线时避免重复初始化) */
    if (initSpiBus) {
        _spi.begin();
    }

    delay(10);
    softwareReset();
    delay(5);

    return true;
}

void AD5761::hardwareReset() {
    if (_resetPin < 0) return;
    digitalWrite(_resetPin, LOW);
    delayMicroseconds(100);   /* t_RESET 最小 100ns */
    digitalWrite(_resetPin, HIGH);
    delay(5);                 /* 物回内部寄存器稳定 */
}

void AD5761::softwareReset() {
    /* 写入命令 0x5 + 特定数据 0x1234 触发软件复位 */
    writeRegister(AD5761_CMD_SOFTWARE_RESET, AD5761_RESET_KEY);
}

bool AD5761::setOutputRange(AD5761_OutputRange range, bool useInternalRef) {
    _range = range;
    _useIntRef = useInternalRef;

    uint32_t ctrl = 0;

    /* 输出范围编码 OVENC (D8~D6) */
    ctrl |= AD5761_CTRL_OVEN((uint32_t)range);

    /* 双极性位:AD5761 中单极性模式 BIPOLAR=1,双极性 BIPOLAR=0 */
    switch (range) {
        case RANGE_UNIPOLAR_10V:
        case RANGE_UNIPOLAR_5V:
            ctrl |= AD5761_CTRL_BIPOLAR;   /* 单极性 */
            break;
        case RANGE_BIPOLAR_10V:
        case RANGE_BIPOLAR_5V:
        case RANGE_BIPOLAR_2_5V:
        default:
            /* 双极性: BIPOLAR 位保持 0 */
            break;
    }

    /* 内部基准使能 */
    if (useInternalRef) {
        ctrl |= AD5761_CTRL_INTREF;
    }

    writeControlRegister(ctrl);
    return true;
}

void AD5761::enableInternalReference(bool enable) {
    _useIntRef = enable;
    /* 通过专用命令快速切换内部基准 */
    writeRegister(AD5761_CMD_SETUP_INTERNAL_REFERENCE, enable ? 0x0001 : 0x0000);
}

bool AD5761::setVoltage(float voltage) {
    /* 钳位到当前输出范围 */
    float vMin, vMax;
    getRangeLimits(vMin, vMax);
    if (voltage < vMin) voltage = vMin;
    if (voltage > vMax) voltage = vMax;

    uint16_t code = voltageToCode(voltage);
    writeAndUpdateDAC(code);
    return true;
}

void AD5761::writeInputRegister(uint16_t code) {
    writeRegister(AD5761_CMD_WRITE_INPUT_REGISTER_ONLY, code);
}

void AD5761::writeAndUpdateDAC(uint16_t code) {
    writeRegister(AD5761_CMD_WRITE_AND_UPDATE_DAC, code);
}

void AD5761::updateDAC() {
    if (_ldacPin < 0) return;
    /* LDAC 下降沿触发:输入寄存器 → DAC 寄存器 */
    digitalWrite(_ldacPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_ldacPin, LOW);
}

void AD5761::setPowerDownMode(AD5761_PowerDownMode mode) {
    uint32_t ctrl = AD5761_CTRL_PD((uint32_t)mode);
    writeControlRegister(ctrl);
}

void AD5761::enableBrownoutReset(bool enable) {
    uint32_t ctrl = enable ? AD5761_CTRL_ERB : 0;
    writeControlRegister(ctrl);
}

void AD5761::enableThermalShutdown(bool enable) {
    uint32_t ctrl = enable ? AD5761_CTRL_ETS : 0;
    writeControlRegister(ctrl);
}

void AD5761::getRangeLimits(float &vMin, float &vMax) const {
    switch (_range) {
        case RANGE_BIPOLAR_10V:   vMin = -10.0f;  vMax =  10.0f;  break;
        case RANGE_UNIPOLAR_10V:  vMin =   0.0f;  vMax =  10.0f;  break;
        case RANGE_BIPOLAR_5V:    vMin =  -5.0f;  vMax =   5.0f;  break;
        case RANGE_UNIPOLAR_5V:   vMin =   0.0f;  vMax =   5.0f;  break;
        case RANGE_BIPOLAR_2_5V:  vMin =  -2.5f;  vMax =  2.5f;  break;
        default:                  vMin = -10.0f;  vMax =  10.0f;  break;
    }
}

uint16_t AD5761::voltageToCode(float voltage) const {
    float vMin, vMax;
    getRangeLimits(vMin, vMax);
    float span = vMax - vMin;

    /* code = (Vout - Vmin) / span * 65535 */
    float normalized = (voltage - vMin) / span;   /* 0.0 ~ 1.0 */
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    /* 加 0.5 做四舍五入,提升精度 */
    uint32_t code = (uint32_t)(normalized * 65535.0f + 0.5f);
    if (code > 65535) code = 65535;
    return (uint16_t)code;
}

float AD5761::codeToVoltage(uint16_t code) const {
    float vMin, vMax;
    getRangeLimits(vMin, vMax);
    float span = vMax - vMin;

    float normalized = (float)code / 65535.0f;
    return vMin + normalized * span;
}

void AD5761::writeRegister(uint8_t cmd, uint16_t data) {
    /* AD5761 命令帧: [C3 C2 C1 C0] [D15 ... D0] 共 24 bit
     * 采用 MSB 优先,先发命令字节,再发两个数据字节 */
    uint8_t cmdByte = (uint8_t)(cmd & 0x0F);
    uint8_t dataHi  = (uint8_t)((data >> 8) & 0xFF);
    uint8_t dataLo  = (uint8_t)(data & 0xFF);

    _spi.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    _spi.transfer(cmdByte);
    _spi.transfer(dataHi);
    _spi.transfer(dataLo);

    digitalWrite(_csPin, HIGH);
    _spi.endTransaction();
}

void AD5761::writeControlRegister(uint32_t value) {
    /* 控制寄存器写: 命令 = 0x2, 数据 = 16-bit 控制字 */
    writeRegister(AD5761_CMD_WRITE_CONTROL_REGISTER, (uint16_t)(value & 0xFFFF));
}

/* ================================================================== *
 *  多芯片管理 AD5761Array 实现
 * ================================================================== */
AD5761Array::AD5761Array(const uint8_t *syncPins, uint8_t count,
                         SPIClass &spi,
                         uint32_t spiFreq,
                         int8_t ldacPin,
                         int8_t clrPin,
                         int8_t resetPin)
    : _dacs(nullptr),
      _count(count),
      _ldacPin(ldacPin),
      _clrPin(clrPin),
      _resetPin(resetPin) {
    if (count == 0 || syncPins == nullptr) return;

    /* 动态创建 count 个 AD5761 对象,共用 SPI,各自独立片选 */
    _dacs = new AD5761 *[count];
    for (uint8_t i = 0; i < count; i++) {
        /* 各芯片不单独持有 LDAC/CLR/RESET,由 Array 统一控制 */
        _dacs[i] = new AD5761(syncPins[i], spi, spiFreq,
                              /*ldac=*/-1, /*clr=*/-1, /*reset=*/-1);
    }
}

AD5761Array::~AD5761Array() {
    if (_dacs == nullptr) return;
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) {
            delete _dacs[i];
            _dacs[i] = nullptr;
        }
    }
    delete[] _dacs;
    _dacs = nullptr;
}

bool AD5761Array::begin() {
    /* 配置共用 LDAC 引脚(默认透明模式,等待 latchAll() 触发同步) */
    if (_ldacPin >= 0) {
        pinMode(_ldacPin, OUTPUT);
        /* 默认拉高 = 保持输入寄存器内容,不立即更新输出。
         * 这样 prepareVoltage*() 写入后需 latchAll() 才会更新。 */
        digitalWrite(_ldacPin, HIGH);
    }

    /* 配置共用 CLR / RESET 引脚 */
    if (_clrPin >= 0) {
        pinMode(_clrPin, OUTPUT);
        digitalWrite(_clrPin, HIGH);
    }
    if (_resetPin >= 0) {
        pinMode(_resetPin, OUTPUT);
        digitalWrite(_resetPin, HIGH);
    }

    /* 逐个初始化:仅第一个初始化 SPI 总线 */
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] == nullptr) continue;
        _dacs[i]->begin(/*initSpiBus=*/(i == 0));
    }
    return true;
}

void AD5761Array::hardwareResetAll() {
    if (_resetPin < 0) return;
    digitalWrite(_resetPin, LOW);
    delayMicroseconds(100);
    digitalWrite(_resetPin, HIGH);
    delay(5);
}

void AD5761Array::softwareResetAll() {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) _dacs[i]->softwareReset();
    }
}

void AD5761Array::setOutputRangeAll(AD5761_OutputRange range, bool useInternalRef) {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) _dacs[i]->setOutputRange(range, useInternalRef);
    }
}

bool AD5761Array::setOutputRange(uint8_t index, AD5761_OutputRange range, bool useInternalRef) {
    if (index >= _count || _dacs[index] == nullptr) return false;
    _dacs[index]->setOutputRange(range, useInternalRef);
    return true;
}

void AD5761Array::setVoltageAll(float voltage) {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) _dacs[i]->setVoltage(voltage);
    }
}

void AD5761Array::prepareVoltageAll(float voltage) {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) {
            uint16_t code = _dacs[i]->voltageToCode(voltage);
            _dacs[i]->writeInputRegister(code);
        }
    }
}

bool AD5761Array::setVoltage(uint8_t index, float voltage) {
    if (index >= _count || _dacs[index] == nullptr) return false;
    _dacs[index]->setVoltage(voltage);
    return true;
}

bool AD5761Array::prepareVoltage(uint8_t index, float voltage) {
    if (index >= _count || _dacs[index] == nullptr) return false;
    uint16_t code = _dacs[index]->voltageToCode(voltage);
    _dacs[index]->writeInputRegister(code);
    return true;
}

void AD5761Array::setVoltages(const float *voltages) {
    if (voltages == nullptr) return;
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) _dacs[i]->setVoltage(voltages[i]);
    }
}

void AD5761Array::prepareVoltages(const float *voltages) {
    if (voltages == nullptr) return;
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) {
            uint16_t code = _dacs[i]->voltageToCode(voltages[i]);
            _dacs[i]->writeInputRegister(code);
        }
    }
}

void AD5761Array::setPowerDownModeAll(AD5761_PowerDownMode mode) {
    for (uint8_t i = 0; i < _count; i++) {
        if (_dacs[i] != nullptr) _dacs[i]->setPowerDownMode(mode);
    }
}

AD5761 *AD5761Array::getDAC(uint8_t index) {
    if (index >= _count) return nullptr;
    return _dacs[index];
}

void AD5761Array::pulseLDAC() {
    if (_ldacPin < 0) return;
    digitalWrite(_ldacPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_ldacPin, LOW);
}

void AD5761Array::latchAll() {
    /* 触发共用 LDAC 下降沿,所有芯片同步将输入寄存器内容更新到 DAC 输出 */
    pulseLDAC();
}