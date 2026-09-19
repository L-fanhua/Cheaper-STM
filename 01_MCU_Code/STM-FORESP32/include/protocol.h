/**
 * @file protocol.h
 * @brief STM 上位机通信协议 v1.0 — 常量与定义
 *
 * 物理层: USB CDC (虚拟串口), 小端字节序, IEEE754 float32
 *
 * 帧格式:
 *   HEADER(2) LENGTH(2) CMD(1) PAYLOAD(N) CRC16(2) TAIL(1)
 *   HEADER = 0x55 0xAA
 *   LENGTH = CMD + PAYLOAD + CRC 的字节数 = 3 + N  (不含帧头/长度/帧尾)
 *   CRC16  = CRC-16/MODBUS, 覆盖 CMD + PAYLOAD
 *   TAIL   = 0x0A
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ================================================================== *
 *  帧标记
 * ================================================================== */
#define FRAME_HEADER_0   0x55
#define FRAME_HEADER_1   0xAA
#define FRAME_TAIL       0x0A

#define LENGTH_MIN       3     /* CMD(1) + CRC(2), 无 payload */
#define LENGTH_MAX       1027  /* CMD(1) + payload(1024) + CRC(2) */
#define PAYLOAD_MAX      1024

/* ================================================================== *
 *  工作模式
 * ================================================================== */
enum WorkMode : uint8_t {
    MODE_IDLE      = 0,
    MODE_FEEDBACK  = 1,
    MODE_SCANNING  = 2,
    MODE_APPROACH  = 3,
};

/* ================================================================== *
 *  上位机 → 下位机 命令码
 * ================================================================== */
#define CMD_PING            0x01
#define CMD_ESTOP           0x02
#define CMD_SET_MODE        0x10
#define CMD_SET_SETPOINT    0x11
#define CMD_SET_PID         0x12
#define CMD_SET_Z_LIMITS    0x13
#define CMD_SET_PID_FREQ    0x14
#define CMD_SET_OVERCURRENT 0x15
#define CMD_MOVE_XYZ        0x20
#define CMD_SET_Z           0x21
#define CMD_RETRACT_Z       0x22
#define CMD_SET_BIAS        0x23   /* 设置样品偏压电压 */
#define CMD_STEP_MOTOR      0x24   /* 手动步进电机控制 */
#define CMD_SET_SCAN_MODE   0x25   /* 设置扫描模式 (恒流/恒高) */
#define CMD_SCAN_START      0x30
#define CMD_SCAN_STOP       0x31
#define CMD_SCAN_PAUSE      0x32
#define CMD_SCAN_RESUME     0x33
#define CMD_APPROACH_START  0x40
#define CMD_APPROACH_STOP   0x41
#define CMD_SET_CALIB       0x50
#define CMD_SAVE_CALIB      0x51
#define CMD_LOAD_CALIB      0x52
#define CMD_GET_STATUS      0x60
#define CMD_GET_FW_VERSION  0x61   /* 查询固件版本号 */

/* ================================================================== *
 *  下位机 → 上位机 命令码
 * ================================================================== */
#define CMD_PONG            0x81
#define CMD_FW_VERSION      0x82   /* 固件版本号 (ASCII) */
#define CMD_STATUS          0x90
#define CMD_SCAN_ROW        0x91
#define CMD_SCAN_END        0x92
#define CMD_APPROACH_DATA   0x93
#define CMD_APPROACH_DONE   0x94
#define CMD_ERROR           0xA0
#define CMD_ACK             0xB0
#define CMD_NACK            0xB1

/* ================================================================== *
 *  扫描模式 (SET_SCAN_MODE)
 * ================================================================== */
enum ScanMode : uint8_t {
    SCAN_MODE_CONSTANT_CURRENT  = 0,   /* 恒流: PID 开, z 随表面起伏调节 (默认) */
    SCAN_MODE_CONSTANT_HEIGHT   = 1,   /* 恒高: PID 关, z 固定 (仅适用平坦表面) */
};

/* ================================================================== *
 *  扫描方向
 * ================================================================== */
enum ScanDirection : uint8_t {
    SCAN_FORWARD      = 0,
    SCAN_BACKWARD     = 1,
    SCAN_BIDIRECTIONAL = 2,
};

/* ================================================================== *
 *  步进电机方向 (STEP_MOTOR)
 * ================================================================== */
enum StepDir : uint8_t {
    STEP_FORWARD  = 0,   /* 前进 (向样品) */
    STEP_BACKWARD = 1,   /* 后退 (远离样品) */
};

/* ================================================================== *
 *  Approach 阶段
 * ================================================================== */
enum ApproachPhase : uint8_t {
    APPROACH_COARSE = 0,
    APPROACH_FINE   = 1,
};

/* ================================================================== *
 *  硬件错误码 (err 字段 / ERROR 帧 err_code)
 * ================================================================== */
#define ERR_NONE                0x00
#define ERR_OVERCURRENT         0x01
#define ERR_Z_OVER_HIGH         0x02
#define ERR_Z_OVER_LOW          0x03
#define ERR_COMM_TIMEOUT        0x04
#define ERR_ADC_FAULT           0x05
#define ERR_DAC_FAULT           0x06
#define ERR_STEP_OVERFLOW       0x07
#define ERR_FINE_APPROACH_FAIL  0x08
#define ERR_SCAN_ABORT          0x09
#define ERR_MODE_INVALID        0x0A
#define ERR_PARAM_OUT_OF_RANGE  0x0B
#define ERR_NVS_WRITE           0x0C
#define ERR_NVS_READ            0x0D
#define ERR_I2C_FAULT           0x0E
#define ERR_SPI_FAULT           0x0F
#define ERR_OVERTEMP            0x10
#define ERR_TASK_WDT            0x11
#define ERR_LOOP_UNSTABLE       0x12
#define ERR_SCAN_TIMEOUT        0x13

/* ================================================================== *
 *  协议层错误码 (仅记录, 不上报)
 * ================================================================== */
#define PROTO_ERR_HEADER          0x80
#define PROTO_ERR_TAIL_MISSING    0x81
#define PROTO_ERR_CRC_MISMATCH    0x82
#define PROTO_ERR_LENGTH_INVALID  0x83
#define PROTO_ERR_CMD_UNKNOWN     0x84
#define PROTO_ERR_PAYLOAD_SIZE    0x85

/* ================================================================== *
 *  NACK 拒绝原因码
 * ================================================================== */
#define NACK_INVALID_TRANSITION    0x01
#define NACK_PARAM_OUT_OF_RANGE    0x02
#define NACK_BUSY                  0x03
#define NACK_NOT_IN_FEEDBACK_MODE  0x04
#define NACK_NOT_IN_IDLE_MODE      0x05
#define NACK_CALIB_NOT_LOADED      0x06
#define NACK_HARDWARE_FAULT        0x07
#define NACK_UNKNOWN_CMD           0x08
#define NACK_INTERNAL_ERROR        0x09

/* ================================================================== *
 *  数值范围约束
 * ================================================================== */
#define SETPOINT_MIN_NA      0.01f
#define SETPOINT_MAX_NA      100.0f
#define OVERCURRENT_MIN_NA   1.0f
#define OVERCURRENT_MAX_NA   200.0f
#define PID_FREQ_MIN_HZ      100UL
#define PID_FREQ_MAX_HZ      50000UL
#define Z_VOLT_MIN           -15.0f
#define Z_VOLT_MAX           15.0f
#define XY_VOLT_MIN          -15.0f
#define XY_VOLT_MAX          15.0f
#define BIAS_VOLT_MIN        -15.0f
#define BIAS_VOLT_MAX        15.0f
#define SCAN_NX_MIN          1
#define SCAN_NX_MAX          1024   /* 单帧限制 */
#define SCAN_NY_MIN          1
#define SCAN_NY_MAX          2048
#define SCAN_SPEED_MIN_PPS   10UL
#define SCAN_SPEED_MAX_PPS   10000UL
#define SCAN_RANGE_MAX_V     30.0f

/* ================================================================== *
 *  Payload 长度期望 (用于校验)
 * ================================================================== */
#define LEN_P_SETPOINT       4
#define LEN_P_PID            12
#define LEN_P_Z_LIMITS       8
#define LEN_P_PID_FREQ       4
#define LEN_P_OVERCURRENT    4
#define LEN_P_MOVE_XYZ       12
#define LEN_P_SET_Z          4
#define LEN_P_RETRACT_Z      6       /* v1.2: steps(u16) + speed(u32), 0=默认速度 */
#define LEN_P_SET_BIAS       4
#define LEN_P_STEP_MOTOR     7
#define LEN_P_SET_SCAN_MODE  1
#define LEN_P_SCAN_START     31
#define LEN_P_APPROACH_START 29     /* v1.4: max_steps 由 u16 改为 u32 (27→29 字节) */
#define LEN_P_SET_CALIB      16

#define LEN_P_STATUS         22
#define LEN_P_SCAN_END       4
#define LEN_P_APPROACH_DATA  9
#define LEN_P_APPROACH_DONE  9
#define LEN_P_ACK            1
#define LEN_P_NACK           2

#endif /* PROTOCOL_H */