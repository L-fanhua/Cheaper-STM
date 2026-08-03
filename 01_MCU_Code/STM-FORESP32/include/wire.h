/**
 * @file wire.h
 * @brief 小端字节序读写辅助 + CRC-16/MODBUS
 *
 * 所有多字节字段均为小端 (Little-Endian), 浮点为 IEEE754 float32。
 */

#ifndef WIRE_H
#define WIRE_H

#include <stdint.h>
#include <string.h>

/* ================================================================== *
 *  读取 (从 buffer 解码, 不对齐安全)
 * ================================================================== */
static inline uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}

static inline int32_t rd_i32(const uint8_t *p) {
    return (int32_t)rd_u32(p);
}

static inline float rd_f32(const uint8_t *p) {
    uint32_t u = rd_u32(p);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

/* ================================================================== *
 *  写入 (编码到 buffer, 不对齐安全)
 * ================================================================== */
static inline void wr_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wr_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void wr_i32(uint8_t *p, int32_t v) {
    wr_u32(p, (uint32_t)v);
}

static inline void wr_f32(uint8_t *p, float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    wr_u32(p, u);
}

/* ================================================================== *
 *  CRC-16/MODBUS
 *    多项式 0x8005 (反向 0xA001)
 *    初始值 0xFFFF, 输入反转, 输出反转, 异或 0x0000
 * ================================================================== */
static inline uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

#endif /* WIRE_H */