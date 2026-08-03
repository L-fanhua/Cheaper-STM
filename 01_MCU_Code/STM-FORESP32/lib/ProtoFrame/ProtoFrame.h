/**
 * @file ProtoFrame.h
 * @brief 协议帧编解码层
 *
 * 帧格式:
 *   HEADER(0x55 0xAA) LENGTH(2,LE) CMD(1) PAYLOAD(N) CRC16(2,LE) TAIL(0x0A)
 *   LENGTH = 3 + N (CMD+PAYLOAD+CRC)
 *   CRC 覆盖 CMD + PAYLOAD
 */

#ifndef PROTO_FRAME_H
#define PROTO_FRAME_H

 #include <stdint.h>
 #include <stddef.h>
 #include "protocol.h"
 #include "wire.h"

/* 单帧最大缓存 (HEADER+LEN+CMD+PAYLOAD+CRC+TAIL) */
#define FRAME_BUF_SIZE  (PAYLOAD_MAX + 16)

/* 接收状态机状态 */
enum RxState : uint8_t {
    RX_SEARCH_HEADER = 0,
    RX_HEADER_1      = 1,
    RX_READ_LENGTH   = 2,
    RX_READ_BODY     = 3,
};

/* 解析结果 */
enum RxResult : uint8_t {
    RX_OK             = 0,  /* 成功解析出一帧 */
    RX_NEED_MORE      = 1,  /* 需要更多数据 */
    RX_ERR_HEADER     = 2,
    RX_ERR_LENGTH     = 3,
    RX_ERR_TAIL       = 4,
    RX_ERR_CRC        = 5,
    RX_ERR_OVERFLOW   = 6,
};

/**
 * @class RxFrame
 * @brief 接收解析器 (状态机, 逐字节喂入)
 *
 * 用法:
 *   while (Serial.available()) {
 *       RxResult r = rx.feed(Serial.read());
 *       if (r == RX_OK) { dispatch(rx.cmd(), rx.payload(), rx.payloadLen()); }
 *   }
 */
class RxFrame {
public:
    RxFrame();

    /** 喂入 1 字节, 返回非 RX_NEED_MORE 表示一帧完成或出错(出错会自动复位) */
    RxResult feed(uint8_t byte);

    /** 解析成功后, 获取命令码 */
    uint8_t cmd() const { return _buf[0]; }

    /** 解析成功后, 获取 payload 指针 */
    const uint8_t *payload() const { return _buf + 1; }

    /** 解析成功后, 获取 payload 字节数 */
    uint16_t payloadLen() const { return _payloadLen; }

    /** 复位状态机 (出错后自动调用) */
    void reset();

    /* 协议层错误统计计数器 */
    uint32_t cntHeader  = 0;
    uint32_t cntLength  = 0;
    uint32_t cntTail    = 0;
    uint32_t cntCrc     = 0;
    uint32_t cntFrameOk = 0;

private:
    RxState   _state;
    uint8_t   _buf[FRAME_BUF_SIZE];
    uint16_t  _idx;         /* 当前写入位置 */
    uint16_t  _length;      /* LENGTH 字段值 */
    uint16_t  _payloadLen;  /* 解析出的 payload 长度 */
};

/* ================================================================== *
 *  发送编码
 * ================================================================== */

/**
 * @brief 编码一帧到 buf
 * @param cmd     命令码
 * @param payload 负载指针 (可为 nullptr 当 payloadLen==0)
 * @param plen    负载字节数
 * @param buf     输出缓冲 (至少 7 + plen 字节)
 * @return 帧总字节数
 */
static inline uint16_t encodeFrame(uint8_t cmd, const uint8_t *payload, uint16_t plen,
                                   uint8_t *buf) {
    uint16_t length = (uint16_t)(3 + plen);

    uint16_t i = 0;
    buf[i++] = FRAME_HEADER_0;
    buf[i++] = FRAME_HEADER_1;
    wr_u16(&buf[i], length); i += 2;
    /* CMD + PAYLOAD 区域 (CRC 覆盖) */
    buf[i++] = cmd;
    if (plen && payload) {
        for (uint16_t k = 0; k < plen; k++) buf[i++] = payload[k];
    }
    /* CRC 覆盖 CMD + PAYLOAD: buf[4 .. 4+plen] */
    uint16_t crc = crc16_modbus(&buf[4], (size_t)(1 + plen));
    wr_u16(&buf[i], crc); i += 2;
    buf[i++] = FRAME_TAIL;
    return i;
}

#endif /* PROTO_FRAME_H */