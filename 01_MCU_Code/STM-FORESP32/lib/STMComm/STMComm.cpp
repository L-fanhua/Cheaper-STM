/**
 * @file STMComm.cpp
 * @brief 通信命令派发器实现
 */

#include "STMComm.h"
#include "wire.h"
#include "version.h"

/* 通信超时阈值 (ms) */
#define COMM_TIMEOUT_MS  5000

STMComm::STMComm(SysState &state) : _port(nullptr), _sys(state) {}

void STMComm::begin(Stream &port) {
    _port = &port;
    _sys.lastPingMs = millis();
}

/* ================================================================== *
 *  收发主循环
 * ================================================================== */
void STMComm::poll() {
    if (!_port) return;
    while (_port->available()) {
        int c = _port->read();
        if (c < 0) break;
        RxResult r = _rx.feed((uint8_t)c);
        if (r == RX_OK) {
            dispatch(_rx.cmd(), _rx.payload(), _rx.payloadLen());
        }
        /* 出错时 _rx 已自动复位, 静默丢弃 */
    }
}

void STMComm::tick() {
    if (!_port) return;
    uint32_t now = millis();

    /* 通信超时检测 (5s 未收到 PING → ESTOP)
     * v1.1.1: 仅在通信已建立后 (收到过首个 PING/命令) 才检查, 避免启动期误触发 */
    if (_sys.commEstablished &&
        (now - _sys.lastPingMs) > COMM_TIMEOUT_MS &&
        _sys.err == ERR_NONE && !_sys.locked) {
        _sys.emergencyStop(ERR_COMM_TIMEOUT);
        sendError(ERR_COMM_TIMEOUT, "Comm timeout: no PING in 5s");
    }

    /* Approach 状态机驱动 (v1.3): 每个周期执行单步逼近 */
    if (_sys.approachActive) {
        approachTick();
    }

    /* STATUS 周期上报 (50 Hz) */
    if ((now - _lastStatusMs) >= _statusPeriodMs) {
        _lastStatusMs = now;
        sendStatus();
    }
}

/* ================================================================== *
 *  发送辅助
 * ================================================================== */
void STMComm::sendFrame(uint8_t cmd, const uint8_t *payload, uint16_t plen) {
    if (!_port) return;
    uint16_t n = encodeFrame(cmd, payload, plen, _txBuf);
    _port->write(_txBuf, n);
}

void STMComm::sendAck(uint8_t cmd) {
    uint8_t p[1] = { cmd };
    sendFrame(CMD_ACK, p, 1);
}

void STMComm::sendNack(uint8_t cmd, uint8_t reason) {
    uint8_t p[2] = { cmd, reason };
    sendFrame(CMD_NACK, p, 2);
}

void STMComm::sendPong() {
    sendFrame(CMD_PONG, nullptr, 0);
}

void STMComm::sendFwVersion() {
    /* v1.1 新增: 返回 FW_VERSION_STRING (ASCII), 长度不超过 64 */
    const char *ver = FW_VERSION_STRING;
    uint16_t len = 0;
    while (ver[len] && len < 64) len++;
    uint8_t p[64];
    for (uint16_t i = 0; i < len; i++) p[i] = (uint8_t)ver[i];
    sendFrame(CMD_FW_VERSION, p, len);
}

void STMComm::sendStatus() {
    /* 刷新电流读数 (桩: 硬件未连接时返回 0) */
    _sys.cur_I_nA = _sys.hw.adcReadI();

    uint8_t p[LEN_P_STATUS];
    uint16_t i = 0;
    p[i++] = _sys.mode;
    wr_f32(&p[i], _sys.cur_I_nA);     i += 4;
    wr_f32(&p[i], _sys.z_V);          i += 4;
    wr_f32(&p[i], _sys.x_V);          i += 4;
    wr_f32(&p[i], _sys.y_V);          i += 4;
    wr_u32(&p[i], (uint32_t)_sys.step_pos); i += 4;
    p[i++] = _sys.err;
    sendFrame(CMD_STATUS, p, LEN_P_STATUS);
}

void STMComm::sendScanRow(uint16_t yIdx, uint16_t nx, const float *I, const float *z) {
    if (!_port) return;
    if (nx > SCAN_NX_MAX) nx = SCAN_NX_MAX;

    /* 直接在 _txScanBuf 中拼装完整帧:
     *   HEADER(2) LENGTH(2) CMD(1) PAYLOAD(4+8*nx) CRC(2) TAIL(1) */
    uint8_t *buf = _txScanBuf;
    uint16_t plen = (uint16_t)(4 + 8 * nx);
    uint16_t length = (uint16_t)(3 + plen);

    uint16_t i = 0;
    buf[i++] = FRAME_HEADER_0;
    buf[i++] = FRAME_HEADER_1;
    wr_u16(&buf[i], length); i += 2;
    /* CMD + PAYLOAD (CRC 覆盖区起点) */
    uint16_t crcStart = i;
    buf[i++] = CMD_SCAN_ROW;
    wr_u16(&buf[i], yIdx); i += 2;
    wr_u16(&buf[i], nx);   i += 2;
    for (uint16_t k = 0; k < nx; k++) { wr_f32(&buf[i], I[k]); i += 4; }
    for (uint16_t k = 0; k < nx; k++) { wr_f32(&buf[i], z[k]); i += 4; }
    /* CRC 覆盖 CMD + PAYLOAD */
    uint16_t crc = crc16_modbus(&buf[crcStart], (size_t)(1 + plen));
    wr_u16(&buf[i], crc); i += 2;
    buf[i++] = FRAME_TAIL;

    _port->write(buf, i);
}

void STMComm::sendScanEnd(uint32_t totalPoints) {
    uint8_t p[4];
    wr_u32(p, totalPoints);
    sendFrame(CMD_SCAN_END, p, 4);
}

void STMComm::sendApproachData(uint8_t phase, uint32_t stepOrZ, float I_nA) {
    uint8_t p[9];
    p[0] = phase;
    wr_u32(&p[1], stepOrZ);
    wr_f32(&p[5], I_nA);
    sendFrame(CMD_APPROACH_DATA, p, 9);
}

void STMComm::sendApproachDone(uint8_t success, uint32_t totalSteps, float finalI_nA) {
    uint8_t p[9];
    p[0] = success;
    wr_u32(&p[1], totalSteps);
    wr_f32(&p[5], finalI_nA);
    sendFrame(CMD_APPROACH_DONE, p, 9);
}

void STMComm::sendError(uint8_t errCode, const char *msg) {
    uint8_t msgLen = 0;
    while (msg && msg[msgLen] && msgLen < 255) msgLen++;
    uint8_t p[2 + 255];
    p[0] = errCode;
    p[1] = msgLen;
    for (uint8_t i = 0; i < msgLen; i++) p[2 + i] = (uint8_t)msg[i];
    sendFrame(CMD_ERROR, p, (uint16_t)(2 + msgLen));
}

/* ================================================================== *
 *  命令派发
 * ================================================================== */
void STMComm::dispatch(uint8_t cmd, const uint8_t *p, uint16_t n) {
    /* 刷新 PING 时间戳 (任意命令都算通信活跃) */
    _sys.commEstablished = true;
    _sys.lastPingMs = millis();

    /* v1.1.1: auto-unlock COMM_TIMEOUT on PING */
    if (cmd == CMD_PING && _sys.locked && _sys.err == ERR_COMM_TIMEOUT) {
        _sys.unlock();
    }

    switch (cmd) {
        /* ---- 心跳 / 急停 ---- */
        case CMD_PING:
            sendPong();
            return;

        case CMD_ESTOP:
            _sys.emergencyStop(ERR_NONE);
            sendError(ERR_NONE, "ESTOP triggered");
            return;

        /* ---- 模式 / 参数 ---- */
        case CMD_SET_MODE: {
            uint8_t reason = handleSetMode(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_SETPOINT: {
            uint8_t reason = handleSetSetpoint(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_PID: {
            uint8_t reason = handleSetPid(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_Z_LIMITS: {
            uint8_t reason = handleSetZLimits(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_PID_FREQ: {
            uint8_t reason = handleSetPidFreq(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_OVERCURRENT: {
            uint8_t reason = handleSetOvercurrent(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }

        /* ---- 运动 ---- */
        case CMD_MOVE_XYZ: {
            uint8_t reason = handleMoveXyz(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_Z: {
            uint8_t reason = handleSetZ(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_RETRACT_Z: {
            uint8_t reason = handleRetractZ(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_BIAS: {                        /* v1.0 新增 */
            uint8_t reason = handleSetBias(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_STEP_MOTOR: {                      /* v1.0 新增 */
            uint8_t reason = handleStepMotor(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SET_SCAN_MODE: {                   /* v1.0 新增 */
            uint8_t reason = handleSetScanMode(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }

        /* ---- 扫描 ---- */
        case CMD_SCAN_START: {
            uint8_t reason = handleScanStart(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SCAN_STOP:
            /* 桩: 停止扫描, 切回 feedback */
            _sys.mode = MODE_FEEDBACK;
            sendAck(cmd);
            return;
        case CMD_SCAN_PAUSE:
            /* 桩 */
            sendAck(cmd);
            return;
        case CMD_SCAN_RESUME:
            /* 桩 */
            sendAck(cmd);
            return;

        /* ---- Approach ---- */
        case CMD_APPROACH_START: {
            uint8_t reason = handleApproachStart(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_APPROACH_STOP:
            _sys.approachActive = false;   /* v1.3: 停止 approach 状态机 */
            _sys.hw.stepStop();
            _sys.mode = MODE_IDLE;
            sendAck(cmd);
            return;

        /* ---- 标定 ---- */
        case CMD_SET_CALIB: {
            uint8_t reason = handleSetCalib(p, n);
            if (reason) sendNack(cmd, reason);
            else        sendAck(cmd);
            return;
        }
        case CMD_SAVE_CALIB:
            if (_sys.hw.nvsSaveCalib(_sys.calib)) {
                _sys.calib.loaded = true;
                sendAck(cmd);
            } else {
                sendNack(cmd, NACK_INTERNAL_ERROR);
            }
            return;
        case CMD_LOAD_CALIB:
            if (_sys.hw.nvsLoadCalib(_sys.calib)) {
                _sys.calib.loaded = true;
                sendAck(cmd);
            } else {
                /* NVS 无数据, 不算致命, 使用默认值 */
                sendNack(cmd, NACK_CALIB_NOT_LOADED);
            }
            return;

        /* ---- 查询 ---- */
        case CMD_GET_STATUS:
            sendStatus();
            return;
        case CMD_GET_FW_VERSION:           /* v1.1 新增 */
            sendFwVersion();
            return;

        default:
            sendNack(cmd, NACK_UNKNOWN_CMD);
            return;
    }
}

/* ================================================================== *
 *  各命令处理器
 * ================================================================== */
uint8_t STMComm::handleSetMode(const uint8_t *p, uint16_t n) {
    if (n != 1) return NACK_PARAM_OUT_OF_RANGE;
    uint8_t newMode = p[0];
    if (newMode > MODE_APPROACH) return NACK_PARAM_OUT_OF_RANGE;
    return _sys.requestMode(newMode);
}

uint8_t STMComm::handleSetSetpoint(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SETPOINT) return NACK_PARAM_OUT_OF_RANGE;
    float v = rd_f32(p);
    if (v < SETPOINT_MIN_NA || v > SETPOINT_MAX_NA) return NACK_PARAM_OUT_OF_RANGE;
    _sys.setpoint_nA = v;
    return 0;
}

uint8_t STMComm::handleSetPid(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_PID) return NACK_PARAM_OUT_OF_RANGE;
    _sys.pid.Kp = rd_f32(&p[0]);
    _sys.pid.Ki = rd_f32(&p[4]);
    _sys.pid.Kd = rd_f32(&p[8]);
    return 0;
}

uint8_t STMComm::handleSetZLimits(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_Z_LIMITS) return NACK_PARAM_OUT_OF_RANGE;
    float zmin = rd_f32(&p[0]);
    float zmax = rd_f32(&p[4]);
    if (zmin < Z_VOLT_MIN || zmax > Z_VOLT_MAX || zmin >= zmax) {
        return NACK_PARAM_OUT_OF_RANGE;
    }
    _sys.z_min = zmin;
    _sys.z_max = zmax;
    return 0;
}

uint8_t STMComm::handleSetPidFreq(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_PID_FREQ) return NACK_PARAM_OUT_OF_RANGE;
    uint32_t f = rd_u32(p);
    if (f < PID_FREQ_MIN_HZ || f > PID_FREQ_MAX_HZ) return NACK_PARAM_OUT_OF_RANGE;
    _sys.pid_freq_Hz = f;
    return 0;
}

uint8_t STMComm::handleSetOvercurrent(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_OVERCURRENT) return NACK_PARAM_OUT_OF_RANGE;
    float v = rd_f32(p);
    if (v < OVERCURRENT_MIN_NA || v > OVERCURRENT_MAX_NA) return NACK_PARAM_OUT_OF_RANGE;
    _sys.overcurrent_nA = v;
    return 0;
}

uint8_t STMComm::handleMoveXyz(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_MOVE_XYZ) return NACK_PARAM_OUT_OF_RANGE;
    if (_sys.mode != MODE_IDLE) return NACK_NOT_IN_IDLE_MODE;
    if (_sys.locked)            return NACK_HARDWARE_FAULT;
    float x = rd_f32(&p[0]);
    float y = rd_f32(&p[4]);
    float z = rd_f32(&p[8]);
    if (x < XY_VOLT_MIN || x > XY_VOLT_MAX) return NACK_PARAM_OUT_OF_RANGE;
    if (y < XY_VOLT_MIN || y > XY_VOLT_MAX) return NACK_PARAM_OUT_OF_RANGE;
    if (z < Z_VOLT_MIN  || z > Z_VOLT_MAX)  return NACK_PARAM_OUT_OF_RANGE;
    _sys.applyXyz(x, y, z);
    return 0;
}

uint8_t STMComm::handleSetZ(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SET_Z) return NACK_PARAM_OUT_OF_RANGE;
    if (_sys.mode != MODE_IDLE && _sys.mode != MODE_FEEDBACK) {
        return NACK_NOT_IN_IDLE_MODE;  /* 或 feedback */
    }
    if (_sys.locked) return NACK_HARDWARE_FAULT;
    float z = rd_f32(p);
    if (z < _sys.z_min || z > _sys.z_max) return NACK_PARAM_OUT_OF_RANGE;
    _sys.applyZ(z);
    return 0;
}

uint8_t STMComm::handleRetractZ(const uint8_t *p, uint16_t n) {
    /* v1.2: 接受 6 字节(steps+speed) 或 2 字节(steps, 向后兼容) */
    uint16_t steps;
    uint32_t speed;

    if (n == 6) {
        /* v1.2: steps(u16) + speed(u32), speed=0 用默认速度 */
        steps = rd_u16(&p[0]);
        speed = rd_u32(&p[2]);
    } else if (n == 2) {
        /* v1.0 旧布局: 仅 steps(u16), 向后兼容 */
        steps = rd_u16(&p[0]);
        speed = 0;      /* 0 = 默认速度 */
    } else {
        return NACK_PARAM_OUT_OF_RANGE;
    }

    if (steps == 0) return NACK_PARAM_OUT_OF_RANGE;

    /* 步进后退 (粗退) */
    _sys.hw.stepMoveDir(steps, STEP_BACKWARD, speed);
    _sys.step_pos -= steps;
    /* z DAC 全退 */
    _sys.retractZ();
    return 0;
}

uint8_t STMComm::handleScanStart(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SCAN_START) return NACK_PARAM_OUT_OF_RANGE;
    if (_sys.mode != MODE_FEEDBACK) return NACK_NOT_IN_FEEDBACK_MODE;
    if (_sys.locked) return NACK_HARDWARE_FAULT;

    ScanParams s;
    uint16_t i = 0;
    s.cx          = rd_f32(&p[i]); i += 4;
    s.cy          = rd_f32(&p[i]); i += 4;
    s.range_x     = rd_f32(&p[i]); i += 4;
    s.range_y     = rd_f32(&p[i]); i += 4;
    s.nx          = rd_u16(&p[i]); i += 2;
    s.ny          = rd_u16(&p[i]); i += 2;
    s.speed_pps   = rd_u32(&p[i]); i += 4;
    s.direction   = p[i];          i += 1;
    s.settle_ms   = rd_u16(&p[i]); i += 2;
    s.overshoot_V = rd_f32(&p[i]); i += 4;

    /* 参数校验 */
    if (s.nx < SCAN_NX_MIN || s.nx > SCAN_NX_MAX)         return NACK_PARAM_OUT_OF_RANGE;
    if (s.ny < SCAN_NY_MIN || s.ny > SCAN_NY_MAX)         return NACK_PARAM_OUT_OF_RANGE;
    if (s.speed_pps < SCAN_SPEED_MIN_PPS || s.speed_pps > SCAN_SPEED_MAX_PPS)
        return NACK_PARAM_OUT_OF_RANGE;
    if (s.range_x <= 0 || s.range_x > SCAN_RANGE_MAX_V)   return NACK_PARAM_OUT_OF_RANGE;
    if (s.range_y <= 0 || s.range_y > SCAN_RANGE_MAX_V)   return NACK_PARAM_OUT_OF_RANGE;
    if (s.direction > 2)                                  return NACK_PARAM_OUT_OF_RANGE;
    /* 端点电压检查 */
    float xLo = s.cx - s.range_x / 2.0f, xHi = s.cx + s.range_x / 2.0f;
    float yLo = s.cy - s.range_y / 2.0f, yHi = s.cy + s.range_y / 2.0f;
    if (xLo < XY_VOLT_MIN || xHi > XY_VOLT_MAX)           return NACK_PARAM_OUT_OF_RANGE;
    if (yLo < XY_VOLT_MIN || yHi > XY_VOLT_MAX)           return NACK_PARAM_OUT_OF_RANGE;

    _sys.scan = s;
    _sys.mode = MODE_SCANNING;
    /* 实际扫描执行由上层任务驱动 (桩: 此处仅记录参数) */
    return 0;
}

uint8_t STMComm::handleSetCalib(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SET_CALIB) return NACK_PARAM_OUT_OF_RANGE;
    CalibParams c;
    uint16_t i = 0;
    c.tia_R = rd_f32(&p[i]); i += 4;
    c.x_nmV = rd_f32(&p[i]); i += 4;
    c.y_nmV = rd_f32(&p[i]); i += 4;
    c.z_nmV = rd_f32(&p[i]); i += 4;
    if (c.tia_R <= 0)        return NACK_PARAM_OUT_OF_RANGE;
    if (c.x_nmV <= 0 || c.y_nmV <= 0 || c.z_nmV <= 0) return NACK_PARAM_OUT_OF_RANGE;
    c.loaded = true;
    _sys.calib = c;
    return 0;
}

uint8_t STMComm::handleApproachStart(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_APPROACH_START) return NACK_PARAM_OUT_OF_RANGE;
    if (_sys.mode != MODE_APPROACH) return NACK_NOT_IN_IDLE_MODE; /* 需先 SET_MODE(3) */
    if (_sys.locked) return NACK_HARDWARE_FAULT;

    ApproachParams a;
    uint16_t i = 0;
    a.step_pulse     = rd_u16(&p[i]); i += 2;
    a.step_speed_sps = rd_u32(&p[i]); i += 4;
    a.thresh_pre_nA  = rd_f32(&p[i]); i += 4;
    a.thresh_lock_nA = rd_f32(&p[i]); i += 4;
    a.z_start_V      = rd_f32(&p[i]); i += 4;
    a.z_speed_Vps    = rd_f32(&p[i]); i += 4;
    a.max_steps      = rd_u16(&p[i]); i += 2;
    a.retry          = p[i];          i += 1;
    a.microstep      = rd_u16(&p[i]); i += 2;

    if (a.step_speed_sps == 0) return NACK_PARAM_OUT_OF_RANGE;
    if (a.max_steps == 0)     return NACK_PARAM_OUT_OF_RANGE;
    if (a.thresh_pre_nA < 0 || a.thresh_lock_nA < 0) return NACK_PARAM_OUT_OF_RANGE;
    if (a.z_start_V < _sys.z_min || a.z_start_V > _sys.z_max) return NACK_PARAM_OUT_OF_RANGE;

    _sys.approach = a;
    /* v1.3: 启动 approach 状态机 */
    _sys.approachActive = true;
    _sys.approachPhase = APPROACH_COARSE;
    _sys.approachTotalSteps = 0;
    _sys.approachCurZ = a.z_start_V;
    _sys.applyZ(a.z_start_V);
    _sys.approachLastTickMs = millis();
    return 0;
}

/* ================================================================== *
 *  v1.0 新增命令处理器
 * ================================================================== */

/* SET_BIAS (0x23): 设置样品偏压电压
 *   payload: bias_V (f32, V), 范围 [-15, 15] */
uint8_t STMComm::handleSetBias(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SET_BIAS) return NACK_PARAM_OUT_OF_RANGE;
    float v = rd_f32(p);
    if (v < BIAS_VOLT_MIN || v > BIAS_VOLT_MAX) return NACK_PARAM_OUT_OF_RANGE;
    _sys.applyBias(v);
    return 0;
}

/* STEP_MOTOR (0x24): 手动步进电机控制
 *   payload: steps(u16) dir(u8) speed_sps(u32), 共 6 字节
 *   dir: 0=forward(向样品), 1=backward(远离样品) */
uint8_t STMComm::handleStepMotor(const uint8_t *p, uint16_t n) {
    /* v1.1.5: 兼容两种 payload 布局:
     *   7 字节: steps(u16) + dir(u8) + speed(u32)  [完整布局]
     *   6 字节: steps(u16) + speed(u32), dir 默认 forward  [简化布局] */
    uint16_t steps;
    uint8_t  dir;
    uint32_t speed;

    if (n == 7) {
        steps = rd_u16(&p[0]);
        dir   = p[2];
        speed = rd_u32(&p[3]);
    } else if (n == 6) {
        steps = rd_u16(&p[0]);
        dir   = STEP_FORWARD;      /* 默认前进 */
        speed = rd_u32(&p[2]);
    } else {
        return NACK_PARAM_OUT_OF_RANGE;
    }

    /* 扫描/approach 进行中禁止手动步进 */
    if (_sys.mode == MODE_SCANNING) return NACK_BUSY;
    if (_sys.locked) return NACK_HARDWARE_FAULT;

    if (steps == 0)    return NACK_PARAM_OUT_OF_RANGE;
    if (dir > 1)       return NACK_PARAM_OUT_OF_RANGE;
    if (speed == 0)    return NACK_PARAM_OUT_OF_RANGE;

    _sys.hw.stepMoveDir(steps, dir, speed);
    /* 更新累计步数 (带符号: forward +, backward -) */
    if (dir == STEP_FORWARD)  _sys.step_pos += steps;
    else                      _sys.step_pos -= steps;
    return 0;
}

/* SET_SCAN_MODE (0x25): 设置扫描模式 (恒流/恒高)
 *   payload: mode(u8), 0=恒流(PID开), 1=恒高(PID关,z固定) */
uint8_t STMComm::handleSetScanMode(const uint8_t *p, uint16_t n) {
    if (n != LEN_P_SET_SCAN_MODE) return NACK_PARAM_OUT_OF_RANGE;
    uint8_t m = p[0];
    if (m > SCAN_MODE_CONSTANT_HEIGHT) return NACK_PARAM_OUT_OF_RANGE;
    _sys.scan_mode = m;
    return 0;
}

/* ================================================================== *
 *  Approach 状态机 (v1.3)
 *
 *  两段式逼近:
 *    PHASE_COARSE: 步进电机粗逼近, 达 thresh_pre → 切 PHASE_FINE
 *    PHASE_FINE:   DAC 细逼近, 达 thresh_lock → 成功; z 达限 → 失败
 * ================================================================== */

/* 粗逼近: 每次执行 step_pulse 步, 等待 1ms, 读电流, 上报 */
static void approachStepCoarse(SysState &_sys, STMComm *comm) {
    const ApproachParams &a = _sys.approach;

    /* 1. 步进 step_pulse 步 (向样品) */
    _sys.hw.stepMoveDir(a.step_pulse, STEP_FORWARD, a.step_speed_sps);
    _sys.approachTotalSteps += a.step_pulse;
    _sys.step_pos += a.step_pulse;

    /* 2. 等待稳定 */
    delay(1);

    /* 3. 读电流 */
    float I = _sys.hw.adcReadI();
    _sys.cur_I_nA = I;

    /* 4. 上报 APPROACH_DATA */
    comm->sendApproachData(APPROACH_COARSE, _sys.approachTotalSteps, I);

    /* 5. 检查切换条件 */
    if (I >= a.thresh_pre_nA) {
        _sys.approachPhase = APPROACH_FINE;
        _sys.approachCurZ = a.z_start_V;
        _sys.applyZ(a.z_start_V);
        _sys.approachLastTickMs = millis();
        return;
    }

    /* 6. 检查最大步数 */
    if (_sys.approachTotalSteps >= a.max_steps) {
        /* 失败 */
        _sys.approachActive = false;
        _sys.mode = MODE_IDLE;
        comm->sendApproachDone(0, _sys.approachTotalSteps, I);
    }
}

/* 细逼近: z DAC 增加, 读电流, 上报 */
static void approachStepFine(SysState &_sys, STMComm *comm) {
    const ApproachParams &a = _sys.approach;

    /* 计算时间增量 (s) */
    uint32_t now = millis();
    float dt = (float)(now - _sys.approachLastTickMs) / 1000.0f;
    _sys.approachLastTickMs = now;
    if (dt <= 0) dt = 0.001f;

    /* 1. z DAC 增加 */
    _sys.approachCurZ += a.z_speed_Vps * dt;
    _sys.applyZ(_sys.approachCurZ);

    /* 2. 读电流 */
    float I = _sys.hw.adcReadI();
    _sys.cur_I_nA = I;

    /* 3. 上报 (z 以 mV 为单位, 有符号) */
    int32_t z_mV = (int32_t)(_sys.approachCurZ * 1000.0f);
    comm->sendApproachData(APPROACH_FINE, (uint32_t)z_mV, I);

    /* 4. 检查锁定 */
    if (I >= a.thresh_lock_nA) {
        /* 成功 */
        _sys.approachActive = false;
        _sys.mode = MODE_IDLE;
        comm->sendApproachDone(1, _sys.approachTotalSteps, I);
        return;
    }

    /* 5. 检查 z 达限 */
    if (_sys.approachCurZ >= _sys.z_max) {
        /* 细逼近失败: 退回 z_start, 步进 retry 步, 回 PHASE_COARSE */
        _sys.approachCurZ = a.z_start_V;
        _sys.applyZ(a.z_start_V);
        _sys.hw.stepMoveDir(a.retry, STEP_FORWARD, a.step_speed_sps);
        _sys.approachTotalSteps += a.retry;
        _sys.step_pos += a.retry;
        _sys.approachPhase = APPROACH_COARSE;
    }
}

void STMComm::approachTick() {
    if (!_sys.approachActive) return;

    if (_sys.approachPhase == APPROACH_COARSE) {
        approachStepCoarse(_sys, this);
    } else {
        approachStepFine(_sys, this);
    }
}
