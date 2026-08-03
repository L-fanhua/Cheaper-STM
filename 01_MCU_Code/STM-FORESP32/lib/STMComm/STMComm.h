/**
 * @file STMComm.h
 * @brief 上位机通信命令派发器
 *
 * 负责: 解析命令 → 校验 → 更新 SysState → 发送响应/数据帧
 * 通过 Stream (USB CDC) 收发。
 */

#ifndef STM_COMM_H
#define STM_COMM_H

#include <Arduino.h>
#include "ProtoFrame.h"
#include "SysState.h"

/**
 * @class STMComm
 * @brief 通信管理器
 */
class STMComm {
public:
    STMComm(SysState &state);

    /** 绑定串口流 (USB CDC) */
    void begin(Stream &port);

    /** 主循环调用: 读取并处理串口数据 */
    void poll();

    /** 周期性任务: STATUS 上报 (50~100Hz), 通信超时检测, Approach 驱动 */
    void tick();

    /* ---- 主动发送辅助 ---- */
    void sendPong();
    void sendFwVersion();           /* v1.1 新增 */
    void sendStatus();
    void sendScanRow(uint16_t yIdx, uint16_t nx, const float *I, const float *z);
    void sendScanEnd(uint32_t totalPoints);
    void sendApproachData(uint8_t phase, uint32_t stepOrZ, float I_nA);
    void sendApproachDone(uint8_t success, uint32_t totalSteps, float finalI_nA);
    void sendError(uint8_t errCode, const char *msg);

    /* 帧统计 */
    uint32_t rxOkCount()   const { return _rx.cntFrameOk; }
    uint32_t rxErrHeader() const { return _rx.cntHeader; }
    uint32_t rxErrLen()    const { return _rx.cntLength; }
    uint32_t rxErrTail()   const { return _rx.cntTail; }
    uint32_t rxErrCrc()    const { return _rx.cntCrc; }

private:
    Stream   *_port;
    SysState &_sys;
    RxFrame   _rx;

    /* 发送 TX 缓冲 (常规小帧) */
    uint8_t _txBuf[FRAME_BUF_SIZE];

    /* 大帧专用缓冲 (SCAN_ROW 最大约 8KB) */
    static const uint16_t SCAN_TX_SIZE = 8 + 8 * SCAN_NX_MAX + 16;
    uint8_t _txScanBuf[SCAN_TX_SIZE];

    /* STATUS 上报节流 */
    uint32_t _lastStatusMs = 0;
    uint32_t _statusPeriodMs = 20;   /* 50 Hz */

    /* ---- 响应发送 ---- */
    void sendAck(uint8_t cmd);
    void sendNack(uint8_t cmd, uint8_t reason);

    /** 发送一帧 (cmd + payload) */
    void sendFrame(uint8_t cmd, const uint8_t *payload, uint16_t plen);

    /* ---- 命令派发 ---- */
    void dispatch(uint8_t cmd, const uint8_t *payload, uint16_t plen);

    /* ---- Approach 状态机驱动 (v1.3) ----
     * 在 tick() 中周期调用, 执行单步逼近逻辑并上报数据 */
    void approachTick();

    /* 各命令处理 (返回 NACK reason, 0=成功) */
    uint8_t handleSetMode(const uint8_t *p, uint16_t n);
    uint8_t handleSetSetpoint(const uint8_t *p, uint16_t n);
    uint8_t handleSetPid(const uint8_t *p, uint16_t n);
    uint8_t handleSetZLimits(const uint8_t *p, uint16_t n);
    uint8_t handleSetPidFreq(const uint8_t *p, uint16_t n);
    uint8_t handleSetOvercurrent(const uint8_t *p, uint16_t n);
    uint8_t handleMoveXyz(const uint8_t *p, uint16_t n);
    uint8_t handleSetZ(const uint8_t *p, uint16_t n);
    uint8_t handleRetractZ(const uint8_t *p, uint16_t n);
    uint8_t handleSetBias(const uint8_t *p, uint16_t n);      /* v1.0 新增 */
    uint8_t handleStepMotor(const uint8_t *p, uint16_t n);    /* v1.0 新增 */
    uint8_t handleSetScanMode(const uint8_t *p, uint16_t n);  /* v1.0 新增 */
    uint8_t handleScanStart(const uint8_t *p, uint16_t n);
    uint8_t handleSetCalib(const uint8_t *p, uint16_t n);
    uint8_t handleApproachStart(const uint8_t *p, uint16_t n);
};

#endif /* STM_COMM_H */