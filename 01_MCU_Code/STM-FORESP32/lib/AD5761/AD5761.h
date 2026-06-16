/**
 * @file AD5761.h
 * @brief AD5761RBRUZ-RL7 16-bit 单通道双极性 DAC 驱动 (Arduino / ESP32)
 *        支持单芯片控制与多芯片(共用 SPI 总线)批量控制
 * @date 2026-06-16
 *
 * AD5761 是 Analog Devices 出品的 16 位精度、单通道、电压输出 DAC,
 * 支持 ±10V / 0~10V / ±5V / 0~5V / ±2.5V 等多种输出范围,
 * 通过 SPI(Mode 1, 最高 50MHz)进行控制,采用 24-bit 命令帧格式。
 *
 * 多芯片应用:多个 AD5761 共用 SCLK / SDI / SDO,各自独立 /SYNC 片选,
 * 可共用 /LDAC 实现同步更新,共用 /RESET、/CLR 实现统一复位/清零。
 */

#ifndef AD5761_H
#define AD5761_H

#include <Arduino.h>
#include <SPI.h>

/* ================================================================== *
 *  AD5761 命令编码 (写入 D23~D20 的 C3~C0)
 * ================================================================== */
#define AD5761_CMD_NOP                       0x0  ///< 空操作
#define AD5761_CMD_WRITE_AND_UPDATE_DAC      0x1  ///< 写入并更新 DAC 寄存器
#define AD5761_CMD_WRITE_CONTROL_REGISTER    0x2  ///< 写控制寄存器
#define AD5761_CMD_WRITE_INPUT_REGISTER_ONLY 0x3  ///< 仅写输入寄存器(不更新输出)
#define AD5761_CMD_RESERVED_4                0x4  ///< 保留
#define AD5761_CMD_SOFTWARE_RESET            0x5  ///< 软件复位(写入特定数据触发)
#define AD5761_CMD_RESERVED_6                0x6  ///< 保留
#define AD5761_CMD_RESERVED_7                0x7  ///< 保留
#define AD5761_CMD_RESERVED_8                0x8  ///< 保留
#define AD5761_CMD_SETUP_INTERNAL_REFERENCE  0x9  ///< 设置内部基准(启用/禁用)
#define AD5761_CMD_RESERVED_A                0xA  ///< 保留
#define AD5761_CMD_RESERVED_B                0xB  ///< 保留
#define AD5761_CMD_RESERVED_C                0xC  ///< 保留
#define AD5761_CMD_RESERVED_D                0xD  ///< 保留
#define AD5761_CMD_RESERVED_E                0xE  ///< 保留
#define AD5761_CMD_NO_OPERATION_READBACK     0xF  ///< 读回操作

/* ================================================================== *
 *  控制寄存器相关位定义
 * ================================================================== */
#define AD5761_CTRL_OFS(x)         (((uint32_t)(x) & 0x7) << 0)   ///< 失调校准 (D2~D0)
#define AD5761_CTRL_GAIN(x)        (((uint32_t)(x) & 0x7) << 3)   ///< 增益校准 (D5~D3)
#define AD5761_CTRL_OVEN(x)        (((uint32_t)(x) & 0x7) << 6)   ///< 输出范围选择 (D8~D6)
#define AD5761_CTRL_BIPOLAR        (1UL << 9)                      ///< 双极性输出 1=单极性 (D9)
#define AD5761_CTRL_ETS            (1UL << 10)                     ///< 使能热关断 (D10)
#define AD5761_CTRL_ERB            (1UL << 11)                     ///< 使能欠压复位 (D11)
#define AD5761_CTRL_I2C_SPI        (1UL << 12)                     ///< 0=SPI 1=I2C (D12)
#define AD5761_CTRL_INTREF         (1UL << 13)                     ///< 内部基准使能 (D13)
#define AD5761_CTRL_PD(x)          (((uint32_t)(x) & 0x3) << 14)   ///< 掉电模式 (D15~D14)

/* 软件复位触发数据 */
#define AD5761_RESET_KEY           0x1234

/**
 * @brief 输出电压范围枚举
 */
enum AD5761_OutputRange {
    RANGE_BIPOLAR_10V   = 0,   ///< ±10 V  (双极性)
    RANGE_UNIPOLAR_10V  = 1,   ///< 0 ~ +10 V (单极性)
    RANGE_BIPOLAR_5V    = 2,   ///< ±5 V   (双极性)
    RANGE_UNIPOLAR_5V   = 3,   ///< 0 ~ +5 V  (单极性)
    RANGE_BIPOLAR_2_5V  = 4,   ///< ±2.5 V (双极性)
};

/**
 * @brief 掉电模式枚举
 */
enum AD5761_PowerDownMode {
    PD_NORMAL      = 0x0,  ///< 正常工作
    PD_1K_TO_GND   = 0x1,  ///< 输出经 1kΩ 接地
    PD_100K_TO_GND = 0x2,  ///< 输出经 100kΩ 接地
    PD_THREE_STATE = 0x3,  ///< 输出高阻态
};

/* ================================================================== *
 *  单芯片驱动类 AD5761
 * ================================================================== */
/**
 * @class AD5761
 * @brief AD5761RBRUZ-RL7 单通道 DAC 驱动类
 *
 * 多芯片共用 SPI 总线时,仅需对第一个实例调用 begin(true) 初始化总线,
 * 其余实例调用 begin(false) 跳过总线初始化以避免冲突。
 */
class AD5761 {
    friend class AD5761Array;

public:
    /**
     * @brief 构造函数
     * @param csPin    片选 (CS / SYNC) 引脚号
     * @param spi      引用的 SPI 实例 (默认 SPI, ESP32 可用 HSPI/FSPI)
     * @param spiFreq  SPI 时钟频率 (Hz),最大 50MHz
     * @param ldacPin  LDAC 引脚号, <0 表示不使用(透明模式)
     * @param clrPin   CLR(清零)引脚号, <0 表示不使用
     * @param resetPin RESET 引脚号, <0 表示不使用
     */
    AD5761(uint8_t csPin,
           SPIClass &spi = SPI,
           uint32_t spiFreq = 1000000UL,
           int8_t ldacPin = -1,
           int8_t clrPin = -1,
           int8_t resetPin = -1);

    /**
     * @brief 初始化引脚与 SPI,必须最先调用
     * @param initSpiBus 是否初始化 SPI 总线。
     *        多芯片共用总线时,仅第一个实例传 true,其余传 false。
     * @return true 初始化成功
     */
    bool begin(bool initSpiBus = true);

    /** @brief 硬件复位(若提供了 RESET 引脚) */
    void hardwareReset();

    /** @brief 软件复位 */
    void softwareReset();

    /**
     * @brief 设置输出电压范围
     * @param range 输出范围,见 AD5761_OutputRange
     * @param useInternalRef 是否启用内部 +4.096V 基准
     * @return true 成功
     */
    bool setOutputRange(AD5761_OutputRange range, bool useInternalRef = false);

    /** @brief 使能/禁用内部基准 */
    void enableInternalReference(bool enable);

    /**
     * @brief 设置并立即输出指定电压
     * @param voltage 目标电压 (V),会被自动钳位到当前范围
     * @return true 成功
     */
    bool setVoltage(float voltage);

    /** @brief 仅写入输入寄存器(不更新输出),配合 updateDAC() 使用 */
    void writeInputRegister(uint16_t code);

    /** @brief 写入并立即更新 DAC 输出 */
    void writeAndUpdateDAC(uint16_t code);

    /** @brief 触发 LDAC,将输入寄存器内容传递到 DAC 寄存器 */
    void updateDAC();

    /** @brief 设置掉电模式 */
    void setPowerDownMode(AD5761_PowerDownMode mode);

    /** @brief 使能欠压复位 (Brownout Reset) */
    void enableBrownoutReset(bool enable);

    /** @brief 使能热关断 (Thermal Shutdown) */
    void enableThermalShutdown(bool enable);

    /** @brief 将电压值转换为 16-bit 码值 */
    uint16_t voltageToCode(float voltage) const;

    /** @brief 将 16-bit 码值转换为电压值 */
    float codeToVoltage(uint16_t code) const;

    /** @brief 获取当前输出范围 */
    AD5761_OutputRange getOutputRange() const { return _range; }

    /** @brief 获取当前输出范围的最小/最大电压 */
    void getRangeLimits(float &vMin, float &vMax) const;

private:
    SPIClass          &_spi;        ///< SPI 实例
    SPISettings        _spiSettings;///< SPI 配置
    uint8_t            _csPin;      ///< CS / SYNC 引脚
    int8_t             _ldacPin;    ///< LDAC 引脚
    int8_t             _clrPin;     ///< CLR 引脚
    int8_t             _resetPin;   ///< RESET 引脚
    AD5761_OutputRange _range;      ///< 当前输出范围
    bool               _useIntRef;  ///< 是否使用内部基准

    /** @brief 底层 24-bit 命令传输 */
    void writeRegister(uint8_t cmd, uint16_t data);

    /** @brief 写控制寄存器 */
    void writeControlRegister(uint32_t value);
};

/* ================================================================== *
 *  多芯片管理类 AD5761Array
 * ================================================================== */
/**
 * @class AD5761Array
 * @brief 多片 AD5761 管理器(共用 SPI 总线,各自独立片选)
 *
 * 典型场景:4 个 AD5761 共用 SCLK / SDI / SDO,SYNC1~SYNC4 独立片选,
 * LDAC / RESET / CLR 可共用以实现同步更新 / 同步复位 / 同步清零。
 *
 * 使用示例:
 * @code
 *   uint8_t syncPins[4] = {5, 6, 7, 8};
 *   AD5761Array dacs(syncPins, 4, SPI, 1000000, /*ldac=*\/ 9,
 *                    /*clr=*\/ -1, /*reset=*\/ 10);
 *   void setup() {
 *       dacs.begin();
 *       dacs.setOutputRangeAll(RANGE_BIPOLAR_10V);
 *       dacs.setVoltageAll(0.0f);          // 同步输出 0V
 *       dacs.setVoltage(2, 2.5f);          // 仅设置第 3 片为 2.5V
 *       dacs.latchAll();                   // 共用 LDAC 同步更新
 *   }
 * @endcode
 */
class AD5761Array {
public:
    /**
     * @brief 构造函数
     * @param syncPins  各芯片 /SYNC 片选引脚数组
     * @param count     芯片数量
     * @param spi       共用的 SPI 实例
     * @param spiFreq   SPI 时钟频率 (Hz)
     * @param ldacPin   共用 /LDAC 引脚 (<0 表示不使用)
     * @param clrPin    共用 /CLR  引脚 (<0 表示不使用)
     * @param resetPin  共用 /RESET引脚 (<0 表示不使用)
     */
    AD5761Array(const uint8_t *syncPins, uint8_t count,
                SPIClass &spi = SPI,
                uint32_t spiFreq = 1000000UL,
                int8_t ldacPin = -1,
                int8_t clrPin = -1,
                int8_t resetPin = -1);

    ~AD5761Array();

    /** @brief 初始化所有芯片(仅初始化一次 SPI 总线) */
    bool begin();

    /** @brief 对所有芯片执行硬件复位(共用 RESET) */
    void hardwareResetAll();

    /** @brief 对所有芯片执行软件复位 */
    void softwareResetAll();

    /**
     * @brief 统一设置所有芯片的输出范围
     * @param range 输出范围
     * @param useInternalRef 是否启用内部基准
     */
    void setOutputRangeAll(AD5761_OutputRange range, bool useInternalRef = false);

    /**
     * @brief 单独设置某一片的输出范围
     * @param index 芯片索引 (0 ~ count-1)
     */
    bool setOutputRange(uint8_t index, AD5761_OutputRange range, bool useInternalRef = false);

    /**
     * @brief 统一设置所有芯片输出相同电压(立即生效)
     * @param voltage 目标电压 (V)
     */
    void setVoltageAll(float voltage);

    /**
     * @brief 统一设置所有芯片输出相同电压到输入寄存器,等待 latchAll() 同步更新
     */
    void prepareVoltageAll(float voltage);

    /**
     * @brief 单独设置某一片的输出电压(立即生效)
     * @param index   芯片索引
     * @param voltage 目标电压 (V)
     * @return true 成功 / false 索引越界
     */
    bool setVoltage(uint8_t index, float voltage);

    /**
     * @brief 单独写入某一片的输入寄存器,等待 latchAll() 同步更新
     */
    bool prepareVoltage(uint8_t index, float voltage);

    /**
     * @brief 分别为每片设置不同电压(立即生效)
     * @param voltages 电压数组,长度需等于芯片数量
     */
    void setVoltages(const float *voltages);

    /**
     * @brief 分别为每片写入输入寄存器,等待 latchAll() 同步更新
     */
    void prepareVoltages(const float *voltages);

    /**
     * @brief 触发共用 LDAC,使所有芯片同步更新输出
     * @note 若未配置 LDAC 引脚,则此函数无操作(各芯片已透明模式即时更新)
     */
    void latchAll();

    /** @brief 统一设置掉电模式 */
    void setPowerDownModeAll(AD5761_PowerDownMode mode);

    /** @brief 获取芯片数量 */
    uint8_t count() const { return _count; }

    /**
     * @brief 获取指定芯片的驱动对象引用(用于高级控制)
     * @return 指针对象,索引越界返回 nullptr
     */
    AD5761 *getDAC(uint8_t index);

private:
    AD5761 **_dacs;     ///< 芯片驱动对象指针数组
    uint8_t  _count;    ///< 芯片数量
    int8_t   _ldacPin;  ///< 共用 LDAC 引脚
    int8_t   _clrPin;   ///< 共用 CLR 引脚
    int8_t   _resetPin; ///< 共用 RESET 引脚

    /** @brief 触发 LDAC 下降沿 */
    void pulseLDAC();
};

#endif /* AD5761_H */