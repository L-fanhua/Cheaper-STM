/**
 * @file SysState.h
 * @brief 系统状态 + 参数 + 模式状态机
 *
 * 持有所有可配置参数与运行时状态, 提供模式切换校验。
 * 硬件相关操作通过 hw 指针回调, 便于通信层独立测试 (硬件桩)。
 */

#ifndef SYS_STATE_H
#define SYS_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/* PID 参数 */
struct PidParams {
    float Kp = 0.1f;
    float Ki = 0.01f;
    float Kd = 0.001f;
};

/* 标定参数 */
struct CalibParams {
    float tia_R = 100000000.0f;   /* 100 MΩ */
    float x_nmV = 10.0f;          /* nm/V */
    float y_nmV = 10.0f;
    float z_nmV = 10.0f;
    bool  loaded = false;         /* 是否已加载 (RAM 或 NVS) */
};

/* 扫描参数 */
struct ScanParams {
    float    cx = 0.0f;
    float    cy = 0.0f;
    float    range_x = 1.0f;
    float    range_y = 1.0f;
    uint16_t nx = 256;
    uint16_t ny = 256;
    uint32_t speed_pps = 1000;
    uint8_t  direction = SCAN_FORWARD;
    uint16_t settle_ms = 10;
    float    overshoot_V = 0.5f;
};

/* Approach 参数 */
struct ApproachParams {
    uint16_t step_pulse     = 1;
    uint32_t step_speed_sps = 100;
    float    thresh_pre_nA  = 0.1f;
    float    thresh_lock_nA = 0.9f;
    float    z_start_V      = 0.0f;
    float    z_speed_Vps    = 1.0f;
    uint32_t max_steps      = 10000;   /* v1.4: 由 u16 扩展为 u32 */
    uint8_t  retry          = 1;
    uint16_t microstep      = 16;
};

/* 扫描模式 (恒流/恒高) — 对应 ScanMode 枚举 */
enum ScanModeField : uint8_t {
    SCAN_MODE_CURRENT = 0,   /* 恒流 */
    SCAN_MODE_HEIGHT  = 1,   /* 恒高 */
};

/* 硬件操作回调接口 (桩: 通信开发阶段不连真实硬件) */
class HardwareOps {
public:
    virtual ~HardwareOps() {}
    /* DAC 输出 (V) */
    virtual void dacSetX(float) {}
    virtual void dacSetY(float) {}
    virtual void dacSetZ(float) {}
    /* 偏压 DAC (V) — v1.0 新增 */
    virtual void dacSetBias(float) {}
    /* ADC 读取 (返回电流 nA) */
    virtual float adcReadI() { return 0.0f; }
    /* 步进电机 */
    virtual void stepMove(uint16_t /*pulse*/) {}
    virtual void stepMoveDir(uint16_t /*pulse*/, uint8_t /*dir*/, uint32_t /*sps*/) {}
    virtual void stepStop() {}
    /* NVS */
    virtual bool nvsSaveCalib(const CalibParams &) { return true; }
    virtual bool nvsLoadCalib(CalibParams &) { return false; }
};

/**
 * @class SysState
 * @brief 全局系统状态
 */
class SysState {
public:
    SysState(HardwareOps &hw);

    /* ---- 当前状态 ---- */
    bool    commEstablished = false; /* 收到首个 PING/命令后置 true, 抑制启动期 COMM_TIMEOUT */
    uint8_t mode = MODE_IDLE;       /* 工作模式 */
    bool    locked = false;         /* ESTOP/致命错误后锁定 */
    uint8_t err = ERR_NONE;         /* 当前错误码 */
    uint32_t lastPingMs = 0;        /* 上次收到 PING 的时间戳 */

    /* ---- 参数 ---- */
    float       setpoint_nA   = 1.0f;
    PidParams   pid;
    float       z_min = -15.0f;
    float       z_max = 15.0f;
    uint32_t    pid_freq_Hz   = 1000;
    float       overcurrent_nA = 10.0f;
    float       bias_V = 0.0f;        /* v1.0 新增: 样品偏压 (V) */
    uint8_t     scan_mode = SCAN_MODE_CURRENT;  /* v1.0 新增: 恒流(0)/恒高(1) */
    CalibParams calib;
    ScanParams  scan;
    ApproachParams approach;

    /* ---- Approach 运行时状态 (v1.3) ---- */
    bool     approachActive = false;     /* approach 是否正在执行 */
    uint8_t  approachPhase = APPROACH_COARSE;  /* 当前阶段 */
    uint32_t approachTotalSteps = 0;     /* 累计粗逼近步数 */
    float    approachCurZ = 0.0f;        /* 细逼近当前 z 电压 */
    uint32_t approachLastTickMs = 0;     /* 上次 approach tick 时间 */

    /* ---- 实时测量值 (供 STATUS 上报) ---- */
    float    cur_I_nA = 0.0f;
    float    z_V = 0.0f;
    float    x_V = 0.0f;
    float    y_V = 0.0f;
    int32_t  step_pos = 0;

    /* ---- 模式状态机 ---- */
    /**
     * @brief 请求切换模式, 返回 NACK 原因码 (0 = 成功)
     */
    uint8_t requestMode(uint8_t newMode);

    /* ---- ESTOP ---- */
    void emergencyStop(uint8_t cause);

    /** @brief 解除锁定 (SET_MODE(idle)) */
    void unlock();

    /* ---- 硬件操作封装 ---- */
    void applyXyz(float x, float y, float z);
    void applyZ(float z);
    void retractZ();
    void applyBias(float v);          /* v1.0 新增 */

    HardwareOps &hw;
};

#endif /* SYS_STATE_H */