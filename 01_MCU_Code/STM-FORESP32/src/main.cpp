/**
 * @file main.cpp
 * @brief STM-FORESP32 固件主程序 (v1.1.4 硬件适配: DAC + ADC + 步进电机)
 *
 * 硬件映射:
 *   AD5761 (SPI3, 只写, 无 MISO):
 *     CS0 -> x  DAC
 *     CS1 -> z  DAC   (注意: z 在 CS1)
 *     CS2 -> y  DAC   (注意: y 在 CS2)
 *     CS3 -> 样品偏压 (bias) DAC
 *
 *   ADS8685 (SPI2/FSPI, 全双工):
 *     测量隧道电流 TIA 输出电压, 换算为电流 I = V_adc / tia_R
 *
 *   步进电机 (TMC2209, STEP/DIR/EN):
 *     EN 低电平有效; 不工作时自动 EN=HIGH 禁用, 防止额外发热
 *
 * 上位机通过 USB CDC 虚拟串口与下位机交互。
 */

#include <Arduino.h>
#include <SPI.h>
#include "SysState.h"
#include "STMComm.h"
#include "AD5761.h"
#include "ADS8685.h"

/* ================================================================== *
 *  ADS8685 引脚定义 (SPI2/FSPI)
 * ================================================================== */
#define ADS8685_SDI_PIN   6      /* MOSI */
#define ADS8685_SDO_PIN   5      /* MISO (SDO0; SDO1 不用) */
#define ADS8685_SCLK_PIN  4      /* SPI 时钟 */
#define ADS8685_CS_PIN    3      /* 片选 / CONVST */
#define ADS8685_RST_PIN   2      /* 硬件复位 */

#define ADS8685_SPI_FREQ  4000000UL

SPIClass adsSPI(FSPI);
ADS8685  g_adc(ADS8685_CS_PIN, adsSPI, ADS8685_SPI_FREQ, ADS8685_RST_PIN);

/* ================================================================== *
 *  步进电机引脚定义 (TMC2209 STEP/DIR/EN)
 *  EN 低电平有效 (TMC2209): LOW=使能, HIGH=禁用
 *  空闲超过 STEPPER_IDLE_TIMEOUT_MS 后自动禁用, 防止发热
 * ================================================================== */
#define STEP_PIN    14
#define DIR_PIN     13
#define EN_PIN      15      /* 低电平有效 */

#define STEPPER_MIN_PULSE_US    10      /* STEP 高/低电平脉宽 (us), TMC2209 需>100ns */
#define STEPPER_EN_SETUP_US     10      /* EN 使能后等待稳定 (us) */
#define STEPPER_DIR_SETUP_US    5       /* DIR 建立时间 (us), STEP 之前 */
#define STEPPER_MIN_INTERVAL_US 1000    /* 默认最小步间间隔 (1ms = 1000sps 上限) */
#define STEPPER_IDLE_TIMEOUT_MS 3000    /* 空闲 3s 后自动禁用 EN */

/* ================================================================== *
 *  DAC 电压 → 16-bit 码值 (±10V 双极性范围)
 * ================================================================== */
static inline uint16_t voltToCode(float v) {
    if (v < -10.0f) v = -10.0f;
    if (v >  10.0f) v =  10.0f;
    float n = (v + 10.0f) / 20.0f;
    uint32_t code = (uint32_t)(n * 65535.0f + 0.5f);
    if (code > 65535) code = 65535;
    return (uint16_t)code;
}

/* ================================================================== *
 *  真实硬件实现
 * ================================================================== */
class RealHardware : public HardwareOps {
public:
    /* ---- 步进电机状态 ---- */
    uint32_t lastMoveMs = 0;     /* 上次步进时间 (用于空闲自动禁用) */

    void initStepper() {
        pinMode(STEP_PIN, OUTPUT);
        pinMode(DIR_PIN, OUTPUT);
        pinMode(EN_PIN, OUTPUT);
        digitalWrite(STEP_PIN, LOW);
        digitalWrite(DIR_PIN, LOW);
        digitalWrite(EN_PIN, HIGH);    /* 初始禁用 */
    }

    /* 使能电机 (移动前调用) */
    inline void enable() {
        digitalWrite(EN_PIN, LOW);
    }
    /* 禁用电机 (空闲时调用, 防发热) */
    inline void disable() {
        digitalWrite(EN_PIN, HIGH);
    }

    /* 生成单个 STEP 脉冲 (不含 DIR 设置, DIR 在 stepMoveDir 开头统一设置一次)
     * TMC2209: STEP 上升沿触发, 需 t_CHIGH > 100ns, t_CLOW > 100ns */
    inline void pulseStep() {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(STEPPER_MIN_PULSE_US);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(STEPPER_MIN_PULSE_US);
        lastMoveMs = millis();
    }

    /* ---- HardwareOps 实现 ---- */

    /* DAC */
    void dacSetX(float v) override {
        ad5761_write(CMD_WR_UPDATE_DAC_REG, voltToCode(v), 0);
    }
    void dacSetZ(float v) override {
        ad5761_write(CMD_WR_UPDATE_DAC_REG, voltToCode(v), 1);
    }
    void dacSetY(float v) override {
        ad5761_write(CMD_WR_UPDATE_DAC_REG, voltToCode(v), 2);
    }
    void dacSetBias(float v) override {
        ad5761_write(CMD_WR_UPDATE_DAC_REG, voltToCode(v), 3);
    }

    /* ADC */
    float adcReadI() override {
        float v = g_adc.readVoltage();
        extern SysState g_sys;
        float R = g_sys.calib.tia_R;
        if (R <= 0.0f) R = 100000000.0f;
        return (v / R) * 1e9f;
    }

    /* ---- 步进电机 ---- */
    /* RETRACT_Z 用: 后退 N 步 (无速度参数, 用默认) */
    void stepMove(uint16_t pulse) override {
        stepMoveDir(pulse, STEP_BACKWARD, 0);
    }

    /* STEP_MOTOR 用: 指定步数/方向/速度 */
    void stepMoveDir(uint16_t pulse, uint8_t dir, uint32_t sps) override {
        if (pulse == 0) return;
        enable();    /* 移动前使能 */
        delayMicroseconds(STEPPER_EN_SETUP_US);   /* EN 稳定后再发 STEP */

        /* v1.1.7: DIR 在循环外统一设置一次, 避免每个脉冲重复设置导致
         *         不同方向下的时序差异 (转速不一致 bug 修复)
         * v1.2.1: DIR 取反 — 因步进电机线序调整, 需软件翻转方向 */
        digitalWrite(DIR_PIN, dir ? LOW : HIGH);
        delayMicroseconds(STEPPER_DIR_SETUP_US);

        /* 步间间隔 (us) = 1e6 / sps; 钳位下限防止过快 */
        uint32_t intervalUs = (sps > 0) ? (1000000UL / sps) : STEPPER_MIN_INTERVAL_US;
        /* 脉冲本身占用 2×PULSE, 确保总间隔不小于此 */
        uint32_t pulseCost = 2 * STEPPER_MIN_PULSE_US;
        if (intervalUs < pulseCost) intervalUs = pulseCost;

        for (uint16_t i = 0; i < pulse; i++) {
            pulseStep();
            /* 补足剩余间隔 (pulseStep 已消耗 pulseCost) */
            if (intervalUs > pulseCost) {
                delayMicroseconds(intervalUs - pulseCost);
            }
        }
        /* 不立即禁用: 由 loop 中的空闲超时检测统一处理 (allow approach 连续步进) */
    }

    void stepStop() override {
        /* 立即禁用 */
        disable();
    }

    /* NVS: 桩 */
    bool nvsSaveCalib(const CalibParams &) override { return true; }
    bool nvsLoadCalib(CalibParams &) override { return false; }
};

/* ================================================================== *
 *  全局对象
 * ================================================================== */
RealHardware g_hw;
SysState     g_sys(g_hw);
STMComm      g_comm(g_sys);

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 2000) {
        delay(10);
    }

    /* DAC (SPI3) */
    ad5761_init();

    /* ADC (SPI2/FSPI) */
    adsSPI.begin(ADS8685_SCLK_PIN, ADS8685_SDO_PIN, ADS8685_SDI_PIN, -1);
    g_adc.begin(false);
    g_adc.setRange(ADS8685_RANGE_BIPOLAR_2_5VREF);

    /* 步进电机 */
    g_hw.initStepper();

    /* 通信 */
    g_comm.begin(Serial);
}

void loop() {
    /* 1. 串口接收 */
    g_comm.poll();

    /* 2. 周期任务 */
    g_comm.tick();

    /* 3. 步进电机空闲自动禁用 (防发热)
     *    超过 STEPPER_IDLE_TIMEOUT_MS 未步进 → EN=HIGH */
    if (g_hw.lastMoveMs != 0 &&
        (millis() - g_hw.lastMoveMs) > STEPPER_IDLE_TIMEOUT_MS) {
        g_hw.disable();
    }

    delay(1);
}