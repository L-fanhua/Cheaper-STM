/**
 * @file ProtoFrame.cpp
 * @brief 接收状态机实现
 */

#include "ProtoFrame.h"

RxFrame::RxFrame() {
    reset();
}

void RxFrame::reset() {
    _state      = RX_SEARCH_HEADER;
    _idx        = 0;
    _length     = 0;
    _payloadLen = 0;
}

RxResult RxFrame::feed(uint8_t b) {
    switch (_state) {
        /* ---- 搜索帧头 ---- */
        case RX_SEARCH_HEADER:
            if (b == FRAME_HEADER_0) {
                _state = RX_HEADER_1;
            }
            return RX_NEED_MORE;

        case RX_HEADER_1:
            if (b == FRAME_HEADER_1) {
                _state  = RX_READ_LENGTH;
                _idx    = 0;
            } else {
                /* 不是 0xAA, 可能是噪声中的 0x55 */
                if (b == FRAME_HEADER_0) {
                    _state = RX_HEADER_1;
                } else {
                    cntHeader++;
                    _state = RX_SEARCH_HEADER;
                }
            }
            return RX_NEED_MORE;

        /* ---- 读 LENGTH (2 字节小端) ---- */
        case RX_READ_LENGTH: {
            /* 复用 _buf[0..1] 暂存 LENGTH 字节 */
            if (_idx == 0) {
                _buf[_idx++] = b;
                return RX_NEED_MORE;
            }
            _buf[_idx++] = b;
            _length = rd_u16(&_buf[0]);

            /* 校验 LENGTH 范围 */
            if (_length < LENGTH_MIN || _length > LENGTH_MAX) {
                cntLength++;
                reset();
                return RX_ERR_LENGTH;
            }
            /* 帧总缓冲需求: 4(HEADER+LEN) 已读, 还需读 LENGTH 字节 (CMD+PAYLOAD+CRC) + 1(TAIL) */
            /* 但我们存储区 _buf 从 CMD 开始, 因此需要 LENGTH + 1 (含 TAIL) 字节 */
            if ((uint32_t)_length + 1 > FRAME_BUF_SIZE) {
                cntLength++;
                reset();
                return RX_ERR_OVERFLOW;
            }
            /* 准备读取 CMD+PAYLOAD+CRC+TAIL */
            _idx   = 0;
            _state = RX_READ_BODY;
            return RX_NEED_MORE;
        }

        /* ---- 读 BODY: CMD + PAYLOAD + CRC + TAIL (共 LENGTH + 1 字节) ---- */
        case RX_READ_BODY: {
            _buf[_idx++] = b;
            /* 需要读取的总字节数 = LENGTH (CMD+PAYLOAD+CRC) + 1 (TAIL) */
            uint16_t need = (uint16_t)(_length + 1);
            if (_idx < need) {
                return RX_NEED_MORE;
            }
            /* 已收满, _buf[0..LENGTH-1] = CMD+PAYLOAD+CRC, _buf[LENGTH] = TAIL */
            uint8_t tail = _buf[_length];
            if (tail != FRAME_TAIL) {
                cntTail++;
                reset();
                return RX_ERR_TAIL;
            }
            /* CRC 覆盖 CMD+PAYLOAD = _buf[0 .. LENGTH-3] */
            uint16_t crcBytes = (uint16_t)(_length - 2); /* 减去 CRC 自身 2 字节 */
            uint16_t crcCalc = crc16_modbus(_buf, crcBytes);
            uint16_t crcRecv = rd_u16(&_buf[crcBytes]);
            if (crcCalc != crcRecv) {
                cntCrc++;
                reset();
                return RX_ERR_CRC;
            }
            /* 成功 */
            _payloadLen = (uint16_t)(crcBytes - 1); /* 减去 CMD 1 字节 */
            cntFrameOk++;
            _state = RX_SEARCH_HEADER;  /* 复位等待下一帧 */
            _idx   = 0;
            return RX_OK;
        }

        default:
            reset();
            return RX_NEED_MORE;
    }
}