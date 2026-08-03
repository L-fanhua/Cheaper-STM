/**
 * @file SysState.cpp
 * @brief 系统状态与模式状态机实现
 */

#include "SysState.h"
#include "Arduino.h"   /* millis() */

SysState::SysState(HardwareOps &h) : hw(h) {}

uint8_t SysState::requestMode(uint8_t newMode) {
    /* 锁定时只允许 idle 解锁 */
    if (locked) {
        if (newMode == MODE_IDLE) {
            unlock();
            return 0;
        }
        return NACK_HARDWARE_FAULT;  /* 锁定中 */
    }

    /* 相同模式: 允许 (幂等) */
    if (newMode == mode) {
        return 0;
    }

    /* 合法切换表 */
    bool ok = false;
    switch (mode) {
        case MODE_IDLE:
            ok = (newMode == MODE_FEEDBACK || newMode == MODE_APPROACH);
            break;
        case MODE_FEEDBACK:
            ok = (newMode == MODE_IDLE || newMode == MODE_SCANNING);
            break;
        case MODE_SCANNING:
            ok = (newMode == MODE_FEEDBACK || newMode == MODE_IDLE);
            /* 实际扫描停止由 SCAN_STOP 处理, 此处允许直接切回 */
            break;
        case MODE_APPROACH:
            ok = (newMode == MODE_IDLE);
            break;
        default:
            ok = false;
    }

    if (!ok) {
        return NACK_INVALID_TRANSITION;
    }

    mode = newMode;
    return 0;
}

void SysState::emergencyStop(uint8_t cause) {
    /* 关闭反馈 */
    if (mode == MODE_FEEDBACK || mode == MODE_SCANNING) {
        /* 反馈环停止 (桩) */
    }
    /* z 全退 */
    retractZ();
    /* 停扫描 / 步进 */
    hw.stepStop();
    /* 进入 idle 锁定 */
    mode = MODE_IDLE;
    err  = cause;
    locked = true;
}

void SysState::unlock() {
    locked = false;
    err = ERR_NONE;
}

void SysState::applyXyz(float x, float y, float z) {
    /* 钳位 */
    if (x < XY_VOLT_MIN) x = XY_VOLT_MIN;
    if (x > XY_VOLT_MAX) x = XY_VOLT_MAX;
    if (y < XY_VOLT_MIN) y = XY_VOLT_MIN;
    if (y > XY_VOLT_MAX) y = XY_VOLT_MAX;
    if (z < z_min) z = z_min;
    if (z > z_max) z = z_max;

    hw.dacSetX(x);
    hw.dacSetY(y);
    hw.dacSetZ(z);
    x_V = x;
    y_V = y;
    z_V = z;
}

void SysState::applyZ(float z) {
    if (z < z_min) z = z_min;
    if (z > z_max) z = z_max;
    hw.dacSetZ(z);
    z_V = z;
}

void SysState::retractZ() {
    hw.dacSetZ(z_min);
    z_V = z_min;
}

void SysState::applyBias(float v) {
    if (v < BIAS_VOLT_MIN) v = BIAS_VOLT_MIN;
    if (v > BIAS_VOLT_MAX) v = BIAS_VOLT_MAX;
    hw.dacSetBias(v);
    bias_V = v;
}