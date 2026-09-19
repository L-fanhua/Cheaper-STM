"""
STM 上位机通信协议层

实现 PROTOCOL.md v1.0 规定的二进制帧编解码。
依赖：仅标准库，无第三方依赖。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

FRAME_HEADER = b"\x55\xAA"   # 帧头 (注意：线序 0x55 0xAA)
FRAME_TAIL = 0x0A            # 帧尾
MAX_PAYLOAD = 1024           # 单帧 payload 上限
MIN_LENGTH = 3               # LENGTH 最小值 = cmd(1) + crc(2)
MAX_LENGTH = MIN_LENGTH + MAX_PAYLOAD


# ---------------------------------------------------------------------------
# CRC-16/MODBUS
# ---------------------------------------------------------------------------

def crc16_modbus(data: bytes, initial: int = 0xFFFF) -> int:
    """
    计算 CRC-16/MODBUS。
    多项式 0xA001 (反转的 0x8005)，初始值 0xFFFF，输入输出均反转。
    """
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# 预计算表 (启动时构建，加速 CRC 计算)
_CRC_TABLE: list[int] = []
def _build_crc_table() -> None:
    for b in range(256):
        crc = b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        _CRC_TABLE.append(crc)
_build_crc_table()


def crc16_modbus_fast(data: bytes, initial: int = 0xFFFF) -> int:
    """查表法加速的 CRC-16/MODBUS。"""
    crc = initial
    for byte in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


# ---------------------------------------------------------------------------
# 命令码枚举
# ---------------------------------------------------------------------------

class Cmd(IntEnum):
    """上位机 → 下位机 命令码"""
    PING            = 0x01
    ESTOP           = 0x02
    SET_MODE        = 0x10
    SET_SETPOINT    = 0x11
    SET_PID         = 0x12
    SET_Z_LIMITS    = 0x13
    SET_PID_FREQ    = 0x14
    SET_OVERCURRENT = 0x15
    MOVE_XYZ        = 0x20
    SET_Z           = 0x21
    RETRACT_Z       = 0x22
    SET_BIAS        = 0x23   # 设置样品偏压电压
    STEP_MOTOR      = 0x24   # 手动步进电机控制
    SET_SCAN_MODE   = 0x25   # 设置扫描模式（恒流/恒高）
    SCAN_START      = 0x30
    SCAN_STOP       = 0x31
    SCAN_PAUSE      = 0x32
    SCAN_RESUME     = 0x33
    APPROACH_START  = 0x40
    APPROACH_STOP   = 0x41
    SET_CALIB       = 0x50
    SAVE_CALIB      = 0x51
    LOAD_CALIB      = 0x52
    GET_STATUS      = 0x60
    GET_FW_VERSION  = 0x61


class Reply(IntEnum):
    """下位机 → 上位机 命令码"""
    PONG          = 0x81
    FW_VERSION    = 0x82
    STATUS        = 0x90
    SCAN_ROW      = 0x91
    SCAN_END      = 0x92
    APPROACH_DATA = 0x93
    APPROACH_DONE = 0x94
    ERROR         = 0xA0
    ACK           = 0xB0
    NACK          = 0xB1


class Mode(IntEnum):
    """设备工作模式"""
    IDLE       = 0
    FEEDBACK   = 1
    SCANNING   = 2
    APPROACH   = 3
    LOCKED     = 0xFF   # ESTOP / 致命错误后锁定


class ScanDirection(IntEnum):
    FORWARD      = 0
    BACKWARD     = 1
    BIDIRECTIONAL = 2


class ScanMode(IntEnum):
    CONSTANT_CURRENT = 0   # 恒流模式（PID 开）
    CONSTANT_HEIGHT  = 1   # 恒高模式（PID 关，z 固定）


class ApproachPhase(IntEnum):
    COARSE = 0   # 步进粗逼近
    FINE   = 1   # DAC 细逼近


# ---------------------------------------------------------------------------
# 错误码
# ---------------------------------------------------------------------------

class ErrCode(IntEnum):
    """硬件错误码 (见 PROTOCOL.md 6.1)"""
    NONE                  = 0x00
    OVERCURRENT           = 0x01
    Z_OVER_HIGH           = 0x02
    Z_OVER_LOW            = 0x03
    COMM_TIMEOUT          = 0x04
    ADC_FAULT             = 0x05
    DAC_FAULT             = 0x06
    STEP_OVERFLOW         = 0x07
    FINE_APPROACH_FAIL    = 0x08
    SCAN_ABORT            = 0x09
    MODE_INVALID          = 0x0A
    PARAM_OUT_OF_RANGE    = 0x0B
    NVS_WRITE             = 0x0C
    NVS_READ              = 0x0D
    I2C_FAULT             = 0x0E
    SPI_FAULT             = 0x0F
    OVERTEMP              = 0x10
    TASK_WDT              = 0x11
    LOOP_UNSTABLE         = 0x12
    SCAN_TIMEOUT          = 0x13


# 致命错误集合：触发后下位机会自动 ESTOP 并进入 LOCKED 状态
FATAL_ERRORS = frozenset({
    ErrCode.OVERCURRENT,
    ErrCode.TASK_WDT,
})

# 需要用户解锁的错误集合：进入 LOCKED 后需 SET_MODE(IDLE) 解锁
LOCKING_ERRORS = frozenset({
    ErrCode.OVERCURRENT,
    ErrCode.Z_OVER_HIGH,
    ErrCode.Z_OVER_LOW,
    ErrCode.COMM_TIMEOUT,
    ErrCode.ADC_FAULT,
    ErrCode.DAC_FAULT,
    ErrCode.I2C_FAULT,
    ErrCode.SPI_FAULT,
    ErrCode.OVERTEMP,
    ErrCode.LOOP_UNSTABLE,
    ErrCode.TASK_WDT,
})


class NackReason(IntEnum):
    """NACK 拒绝原因码 (见 PROTOCOL.md 6.3)"""
    INVALID_TRANSITION    = 0x01
    PARAM_OUT_OF_RANGE    = 0x02
    BUSY                  = 0x03
    NOT_IN_FEEDBACK_MODE  = 0x04
    NOT_IN_IDLE_MODE      = 0x05
    CALIB_NOT_LOADED      = 0x06
    HARDWARE_FAULT        = 0x07
    UNKNOWN_CMD           = 0x08
    INTERNAL_ERROR        = 0x09


class ProtoErr(IntEnum):
    """协议层错误 (仅上位机内部记录，不与 ErrCode 混淆)"""
    HEADER          = 0x80
    TAIL_MISSING    = 0x81
    CRC_MISMATCH    = 0x82
    LENGTH_INVALID  = 0x83
    CMD_UNKNOWN     = 0x84
    PAYLOAD_SIZE    = 0x85


# 错误码 → 可读描述
ERR_DESCRIPTIONS = {
    ErrCode.NONE: "无错误",
    ErrCode.OVERCURRENT: "过流 (tip 撞样品)",
    ErrCode.Z_OVER_HIGH: "z 电压超上限",
    ErrCode.Z_OVER_LOW: "z 电压超下限",
    ErrCode.COMM_TIMEOUT: "通信超时",
    ErrCode.ADC_FAULT: "ADC 故障",
    ErrCode.DAC_FAULT: "DAC 故障",
    ErrCode.STEP_OVERFLOW: "步进超 max_steps",
    ErrCode.FINE_APPROACH_FAIL: "细逼近失败",
    ErrCode.SCAN_ABORT: "扫描异常中止",
    ErrCode.MODE_INVALID: "模式非法",
    ErrCode.PARAM_OUT_OF_RANGE: "参数越界",
    ErrCode.NVS_WRITE: "NVS 写入失败",
    ErrCode.NVS_READ: "NVS 读取失败",
    ErrCode.I2C_FAULT: "I2C 通信故障",
    ErrCode.SPI_FAULT: "SPI 通信故障",
    ErrCode.OVERTEMP: "MCU 过温",
    ErrCode.TASK_WDT: "任务看门狗触发",
    ErrCode.LOOP_UNSTABLE: "PID 反馈环路发散",
    ErrCode.SCAN_TIMEOUT: "扫描点采集超时",
}

NACK_DESCRIPTIONS = {
    NackReason.INVALID_TRANSITION: "模式切换非法",
    NackReason.PARAM_OUT_OF_RANGE: "参数越界",
    NackReason.BUSY: "设备忙",
    NackReason.NOT_IN_FEEDBACK_MODE: "当前非 feedback 模式",
    NackReason.NOT_IN_IDLE_MODE: "当前非 idle 模式",
    NackReason.CALIB_NOT_LOADED: "标定未加载",
    NackReason.HARDWARE_FAULT: "硬件故障",
    NackReason.UNKNOWN_CMD: "未知命令",
    NackReason.INTERNAL_ERROR: "内部错误",
}


# ---------------------------------------------------------------------------
# 异常体系
# ---------------------------------------------------------------------------

class ProtocolError(Exception):
    """协议层基础异常"""

class FrameError(ProtocolError):
    """帧格式错误 (帧头/帧尾/长度)"""
    def __init__(self, kind: ProtoErr, detail: str = ""):
        self.kind = kind
        self.detail = detail
        super().__init__(f"{kind.name}: {detail}" if detail else kind.name)

class CrcError(ProtocolError):
    """CRC 校验失败"""
    def __init__(self, expected: int, got: int):
        self.expected = expected
        self.got = got
        super().__init__(f"CRC mismatch: expected 0x{expected:04X}, got 0x{got:04X}")

class PayloadError(ProtocolError):
    """payload 长度与命令不符"""
    def __init__(self, cmd: int, expected: int, got: int):
        self.cmd = cmd
        self.expected = expected
        self.got = got
        super().__init__(
            f"Payload size mismatch for cmd 0x{cmd:02X}: "
            f"expected {expected}, got {got}"
        )

class UnknownCmdError(ProtocolError):
    """未知命令码"""
    def __init__(self, cmd: int):
        self.cmd = cmd
        super().__init__(f"Unknown command code: 0x{cmd:02X}")


# 设备级异常 (来自下位机 ERROR / NACK)
class DeviceError(Exception):
    """设备错误基类"""
    def __init__(self, code: ErrCode, msg: str = ""):
        self.code = code
        self.msg = msg
        desc = ERR_DESCRIPTIONS.get(code, f"未知错误码 0x{int(code):02X}")
        text = f"[{code.name}] {desc}"
        if msg:
            text += f": {msg}"
        super().__init__(text)

class NackError(DeviceError):
    """下位机拒绝命令"""
    def __init__(self, cmd: int, reason: NackReason):
        self.cmd = cmd
        self.reason = reason
        desc = NACK_DESCRIPTIONS.get(reason, f"未知原因 0x{int(reason):02X}")
        cmd_name = _cmd_name(cmd)
        super().__init__(
            ErrCode.MODE_INVALID if reason == NackReason.INVALID_TRANSITION
            else ErrCode.PARAM_OUT_OF_RANGE if reason == NackReason.PARAM_OUT_OF_RANGE
            else ErrCode.NONE,
            f"NACK for {cmd_name}: {desc}"
        )
        # 修正 message 字段
        self.msg = f"NACK for {cmd_name}: {desc}"

class FatalDeviceError(DeviceError):
    """致命设备错误 (过流/看门狗)，下位机已 ESTOP"""


def _cmd_name(cmd: int) -> str:
    """命令码转可读名称"""
    try:
        return Cmd(cmd).name
    except ValueError:
        try:
            return Reply(cmd).name
        except ValueError:
            return f"0x{cmd:02X}"


# ---------------------------------------------------------------------------
# 数据类：上行 (下位机 → 上位机) 的解析结果
# ---------------------------------------------------------------------------

@dataclass
class Status:
    """STATUS (0x90) 解析结果"""
    mode: Mode
    current_nA: float
    z_V: float
    x_V: float
    y_V: float
    step_pos: int      # 带符号
    err: ErrCode

    @classmethod
    def unpack(cls, payload: bytes) -> "Status":
        if len(payload) != 22:
            raise PayloadError(Reply.STATUS, 22, len(payload))
        mode, cur_I, z, x, y, step, err = struct.unpack("<BffffIB", payload)
        return cls(
            mode=Mode(mode),
            current_nA=cur_I,
            z_V=z,
            x_V=x,
            y_V=y,
            step_pos=struct.unpack("<i", struct.pack("<I", step))[0],  # 转 signed
            err=ErrCode(err),
        )

    @property
    def is_locked(self) -> bool:
        return self.mode == Mode.LOCKED or self.err in LOCKING_ERRORS

    @property
    def is_fatal(self) -> bool:
        return self.err in FATAL_ERRORS


@dataclass
class ScanRow:
    """SCAN_ROW (0x91) 解析结果"""
    y_idx: int
    I_data: list[float]
    z_data: list[float]

    @classmethod
    def unpack(cls, payload: bytes) -> "ScanRow":
        if len(payload) < 8:
            raise PayloadError(Reply.SCAN_ROW, -1, len(payload))
        y_idx, nx = struct.unpack("<HH", payload[:4])
        expected = 8 + 8 * nx
        if len(payload) != expected:
            raise PayloadError(Reply.SCAN_ROW, expected, len(payload))
        I_data = list(struct.unpack(f"<{nx}f", payload[4:4 + 4 * nx]))
        z_data = list(struct.unpack(f"<{nx}f", payload[4 + 4 * nx:]))
        return cls(y_idx=y_idx, I_data=I_data, z_data=z_data)

    @property
    def nx(self) -> int:
        return len(self.I_data)


@dataclass
class ScanEnd:
    """SCAN_END (0x92)"""
    total_points: int

    @classmethod
    def unpack(cls, payload: bytes) -> "ScanEnd":
        if len(payload) != 4:
            raise PayloadError(Reply.SCAN_END, 4, len(payload))
        return cls(total_points=struct.unpack("<I", payload)[0])


@dataclass
class ApproachData:
    """APPROACH_DATA (0x93)"""
    phase: ApproachPhase
    step_or_z: int    # phase=COARSE: 步数; phase=FINE: z 电压 mV (signed)
    current_nA: float

    @classmethod
    def unpack(cls, payload: bytes) -> "ApproachData":
        if len(payload) != 9:
            raise PayloadError(Reply.APPROACH_DATA, 9, len(payload))
        phase, raw_step, I = struct.unpack("<BI f", payload)
        # step_or_z 在 phase=FINE 时是 signed，这里统一按 signed 解
        step = struct.unpack("<i", struct.pack("<I", raw_step))[0]
        return cls(
            phase=ApproachPhase(phase),
            step_or_z=step,
            current_nA=I,
        )


@dataclass
class ApproachDone:
    """APPROACH_DONE (0x94)"""
    success: bool
    total_steps: int
    final_current_nA: float

    @classmethod
    def unpack(cls, payload: bytes) -> "ApproachDone":
        if len(payload) != 9:
            raise PayloadError(Reply.APPROACH_DONE, 9, len(payload))
        success, steps, I = struct.unpack("<BI f", payload)
        return cls(
            success=bool(success),
            total_steps=steps,
            final_current_nA=I,
        )


@dataclass
class ErrorFrame:
    """ERROR (0xA0)"""
    code: ErrCode
    msg: str

    @classmethod
    def unpack(cls, payload: bytes) -> "ErrorFrame":
        if len(payload) < 2:
            raise PayloadError(Reply.ERROR, -1, len(payload))
        code, msg_len = struct.unpack("<BB", payload[:2])
        if len(payload) != 2 + msg_len:
            raise PayloadError(Reply.ERROR, 2 + msg_len, len(payload))
        msg = payload[2:2 + msg_len].decode("ascii", errors="replace")
        return cls(code=ErrCode(code), msg=msg)

    def to_exception(self) -> DeviceError:
        if self.code in FATAL_ERRORS:
            return FatalDeviceError(self.code, self.msg)
        return DeviceError(self.code, self.msg)


@dataclass
class Ack:
    """ACK (0xB0)"""
    cmd: int

    @classmethod
    def unpack(cls, payload: bytes) -> "Ack":
        if len(payload) != 1:
            raise PayloadError(Reply.ACK, 1, len(payload))
        return cls(cmd=payload[0])


@dataclass
class Nack:
    """NACK (0xB1)"""
    cmd: int
    reason: NackReason

    @classmethod
    def unpack(cls, payload: bytes) -> "Nack":
        if len(payload) != 2:
            raise PayloadError(Reply.NACK, 2, len(payload))
        cmd, reason = struct.unpack("<BB", payload)
        return cls(cmd=cmd, reason=NackReason(reason))

    def to_exception(self) -> NackError:
        return NackError(self.cmd, self.reason)


# ---------------------------------------------------------------------------
# 帧编解码
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    """完整的一帧 (CMD + Payload)"""
    cmd: int
    payload: bytes = b""

    def encode(self) -> bytes:
        """编码为完整字节流 (含帧头/长度/CRC/帧尾)"""
        cmd_byte = bytes([self.cmd])
        body = cmd_byte + self.payload
        crc = crc16_modbus_fast(body)
        length = len(body) + 2  # +2 for CRC
        if length < MIN_LENGTH or length > MAX_LENGTH:
            raise FrameError(
                ProtoErr.LENGTH_INVALID,
                f"length {length} out of range [{MIN_LENGTH}, {MAX_LENGTH}]"
            )
        return (
            FRAME_HEADER
            + struct.pack("<H", length)
            + body
            + struct.pack("<H", crc)
            + bytes([FRAME_TAIL])
        )

    @staticmethod
    def decode(buf: bytes) -> "Frame":
        """从完整字节流解析一帧 (不含流式状态机)"""
        if len(buf) < 7:
            raise FrameError(ProtoErr.LENGTH_INVALID, f"frame too short: {len(buf)}")
        if buf[:2] != FRAME_HEADER:
            raise FrameError(ProtoErr.HEADER, f"bad header: {buf[:2].hex()}")
        if buf[-1] != FRAME_TAIL:
            raise FrameError(ProtoErr.TAIL_MISSING, f"bad tail: 0x{buf[-1]:02X}")
        length = struct.unpack("<H", buf[2:4])[0]
        if length < MIN_LENGTH or length > MAX_LENGTH:
            raise FrameError(ProtoErr.LENGTH_INVALID, f"length {length} invalid")
        expected_total = 4 + length + 1  # header(2)+len(2)+body+crc(2 in length)+tail(1)
        # length = cmd(1) + payload + crc(2) → body = length bytes
        # total = 2 (header) + 2 (len) + length + 1 (tail)
        if len(buf) != expected_total:
            raise FrameError(
                ProtoErr.LENGTH_INVALID,
                f"frame size {len(buf)} != expected {expected_total}"
            )
        body = buf[4:4 + length - 2]   # cmd + payload (不含 CRC)
        crc_recv = struct.unpack("<H", buf[4 + length - 2:4 + length])[0]
        crc_calc = crc16_modbus_fast(body)
        if crc_recv != crc_calc:
            raise CrcError(crc_calc, crc_recv)
        cmd = body[0]
        payload = body[1:]
        return Frame(cmd=cmd, payload=payload)


# ---------------------------------------------------------------------------
# 流式解析器：用于串口读取
# ---------------------------------------------------------------------------

class FrameParser:
    """
    流式帧解析器。

    用法：
        parser = FrameParser()
        parser.feed(serial.read(1024))
        while True:
            frame = parser.next_frame()
            if frame is None:
                break
            handle(frame)
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.proto_err_count = {
            ProtoErr.HEADER: 0,
            ProtoErr.TAIL_MISSING: 0,
            ProtoErr.CRC_MISMATCH: 0,
            ProtoErr.LENGTH_INVALID: 0,
        }

    def feed(self, data: bytes) -> None:
        """喂入新数据"""
        if data:
            self._buf.extend(data)

    def _find_header(self) -> int:
        """在 buffer 中查找下一个帧头位置，找不到返回 -1"""
        idx = self._buf.find(FRAME_HEADER)
        return idx

    def next_frame(self) -> Optional[Frame]:
        """
        尝试从 buffer 解析一帧。
        - 解析成功：返回 Frame，并从 buffer 移除对应字节
        - 数据不足：返回 None，等待更多数据
        - 解析失败：跳过错误字节，记录计数，继续尝试
        """
        while True:
            # 1. 找帧头
            idx = self._find_header()
            if idx < 0:
                # 没有帧头，清空 buffer (保留最后 1 字节，防跨包 0x55)
                if len(self._buf) > 1:
                    del self._buf[:-1]
                return None
            if idx > 0:
                # 丢弃帧头前的杂字节
                del self._buf[:idx]

            # 2. 长度字段是否完整
            if len(self._buf) < 4:
                return None

            length = struct.unpack("<H", self._buf[2:4])[0]
            if length < MIN_LENGTH or length > MAX_LENGTH:
                self.proto_err_count[ProtoErr.LENGTH_INVALID] += 1
                # 跳过这个假的帧头字节，继续找下一个
                del self._buf[:1]
                continue

            # 3. 整帧是否完整
            total_len = 4 + length + 1   # header(2) + len(2) + body(length) + tail(1)
            if len(self._buf) < total_len:
                return None

            # 4. 帧尾校验
            if self._buf[total_len - 1] != FRAME_TAIL:
                self.proto_err_count[ProtoErr.TAIL_MISSING] += 1
                del self._buf[:1]
                continue

            # 5. CRC 校验
            body = bytes(self._buf[4:4 + length - 2])
            crc_recv = struct.unpack("<H", self._buf[4 + length - 2:4 + length])[0]
            crc_calc = crc16_modbus_fast(body)
            if crc_recv != crc_calc:
                self.proto_err_count[ProtoErr.CRC_MISMATCH] += 1
                del self._buf[:1]
                continue

            # 6. 解析成功
            cmd = body[0]
            payload = bytes(body[1:])
            del self._buf[:total_len]
            return Frame(cmd=cmd, payload=payload)

    def reset(self) -> None:
        """清空 buffer 和计数器"""
        self._buf.clear()
        for k in self.proto_err_count:
            self.proto_err_count[k] = 0

    @property
    def buffer_len(self) -> int:
        return len(self._buf)


# ---------------------------------------------------------------------------
# 上行命令构造器：构造 Frame 对象
# ---------------------------------------------------------------------------

def _make(cmd: Cmd, payload: bytes = b"") -> Frame:
    """构造上行帧"""
    return Frame(cmd=int(cmd), payload=payload)


def ping() -> Frame:
    return _make(Cmd.PING)

def estop() -> Frame:
    return _make(Cmd.ESTOP)

def set_mode(mode: Mode) -> Frame:
    return _make(Cmd.SET_MODE, bytes([int(mode)]))

def set_setpoint(current_nA: float) -> Frame:
    if not 0.01 <= current_nA <= 100:
        raise ValueError(f"setpoint {current_nA} nA out of [0.01, 100]")
    return _make(Cmd.SET_SETPOINT, struct.pack("<f", current_nA))

def set_pid(kp: float, ki: float, kd: float) -> Frame:
    return _make(Cmd.SET_PID, struct.pack("<fff", kp, ki, kd))

def set_z_limits(z_min_V: float, z_max_V: float) -> Frame:
    if not -15 <= z_min_V < z_max_V <= 15:
        raise ValueError(f"z limits [{z_min_V}, {z_max_V}] invalid")
    return _make(Cmd.SET_Z_LIMITS, struct.pack("<ff", z_min_V, z_max_V))

def set_pid_freq(freq_Hz: int) -> Frame:
    if not 100 <= freq_Hz <= 50000:
        raise ValueError(f"PID freq {freq_Hz} out of [100, 50000]")
    return _make(Cmd.SET_PID_FREQ, struct.pack("<I", freq_Hz))

def set_overcurrent(I_limit_nA: float) -> Frame:
    if not 1 <= I_limit_nA <= 200:
        raise ValueError(f"overcurrent {I_limit_nA} out of [1, 200]")
    return _make(Cmd.SET_OVERCURRENT, struct.pack("<f", I_limit_nA))

def move_xyz(x_V: float, y_V: float, z_V: float) -> Frame:
    for v in (x_V, y_V, z_V):
        if not -15 <= v <= 15:
            raise ValueError(f"voltage {v} out of [-15, 15]")
    return _make(Cmd.MOVE_XYZ, struct.pack("<fff", x_V, y_V, z_V))

def set_z(z_V: float) -> Frame:
    if not -15 <= z_V <= 15:
        raise ValueError(f"z voltage {z_V} out of [-15, 15]")
    return _make(Cmd.SET_Z, struct.pack("<f", z_V))

def retract_z(steps: int, speed_sps: int = 0) -> Frame:
    """RETRACT_Z: 步进电机后退指定步数 + z DAC 设为 z_min

    Payload 布局 (6 字节)：
        偏移 0  长度 2  字段 steps    u16   后退步数 (1~65535)
        偏移 2  长度 4  字段 speed    u32   后退速度 (steps/s)，0=用固件默认速度
    """
    if not 0 <= steps <= 65535:
        raise ValueError(f"steps {steps} out of [0, 65535]")
    if not 0 <= speed_sps <= 100000:
        raise ValueError(f"speed_sps {speed_sps} out of [0, 100000]")
    return _make(Cmd.RETRACT_Z, struct.pack("<HI", steps, speed_sps))

def set_bias(bias_V: float) -> Frame:
    """SET_BIAS: 设置样品偏压电压"""
    return _make(Cmd.SET_BIAS, struct.pack("<f", bias_V))

def step_motor(steps: int, speed_sps: int) -> Frame:
    """STEP_MOTOR: 手动步进电机（steps 正=进，负=退）

    Payload 布局 (6 字节)：
        偏移 0  长度 2  字段 steps    i16   步数，带符号（正=进，负=退）
        偏移 2  长度 4  字段 speed    u32   步进频率 (steps/s)
    """
    if steps == 0:
        raise ValueError("steps must be non-zero")
    if not -32768 <= steps <= 32767:
        raise ValueError(f"steps {steps} out of [-32768, 32767]")
    if not 1 <= speed_sps <= 100000:
        raise ValueError(f"speed_sps {speed_sps} out of [1, 100000]")
    return _make(Cmd.STEP_MOTOR, struct.pack("<hI", steps, speed_sps))

def set_scan_mode(mode: ScanMode) -> Frame:
    """SET_SCAN_MODE: 设置扫描模式"""
    return _make(Cmd.SET_SCAN_MODE, struct.pack("<B", int(mode)))

def scan_start(
    cx_V: float,
    cy_V: float,
    range_x_V: float,
    range_y_V: float,
    nx: int,
    ny: int,
    speed_pps: int,
    direction: ScanDirection = ScanDirection.FORWARD,
    settle_ms: int = 10,
    overshoot_V: float = 0.0,
) -> Frame:
    if not 1 <= nx <= 1024:
        raise ValueError(f"nx {nx} out of [1, 1024]")
    if not 1 <= ny <= 2048:
        raise ValueError(f"ny {ny} out of [1, 2048]")
    if not 10 <= speed_pps <= 10000:
        raise ValueError(f"speed {speed_pps} out of [10, 10000]")
    if not 0 < range_x_V <= 30 or not 0 < range_y_V <= 30:
        raise ValueError("range out of (0, 30]")
    # 校验扫描区域不越 DAC 量程
    if not -15 <= cx_V - range_x_V / 2 or cx_V + range_x_V / 2 > 15:
        raise ValueError("x scan range exceeds DAC limit")
    if not -15 <= cy_V - range_y_V / 2 or cy_V + range_y_V / 2 > 15:
        raise ValueError("y scan range exceeds DAC limit")
    payload = struct.pack(
        "<ff ff HH I B H f",
        cx_V, cy_V, range_x_V, range_y_V,
        nx, ny,
        speed_pps,
        int(direction),
        settle_ms,
        overshoot_V,
    )
    return _make(Cmd.SCAN_START, payload)

def scan_stop() -> Frame:
    return _make(Cmd.SCAN_STOP)

def scan_pause() -> Frame:
    return _make(Cmd.SCAN_PAUSE)

def scan_resume() -> Frame:
    return _make(Cmd.SCAN_RESUME)

def approach_start(
    step_pulse: int,
    step_speed_sps: int,
    thresh_pre_nA: float,
    thresh_lock_nA: float,
    z_start_V: float,
    z_speed_Vps: float,
    max_steps: int,
    retry: int = 5,
    microstep: int = 16,
) -> Frame:
    if not 1 <= step_pulse <= 65535:
        raise ValueError(f"step_pulse {step_pulse} out of [1, 65535]")
    if not 1 <= step_speed_sps <= 100000:
        raise ValueError(f"step_speed {step_speed_sps} out of [1, 100000]")
    if not 0 <= thresh_pre_nA < thresh_lock_nA <= 100:
        raise ValueError("thresh_pre must be < thresh_lock, both in [0, 100]")
    if not -15 <= z_start_V <= 15:
        raise ValueError(f"z_start {z_start_V} out of [-15, 15]")
    if not 0 < z_speed_Vps <= 100:
        raise ValueError(f"z_speed {z_speed_Vps} out of (0, 100]")
    if not 1 <= max_steps <= 1000000:
        raise ValueError(f"max_steps {max_steps} out of [1, 1000000]")
    if microstep not in (1, 2, 4, 8, 16, 32, 64, 128, 256):
        raise ValueError(f"microstep {microstep} invalid")
    # 长度自适应编码 (v1.1)：
    # - max_steps <= 65535 → 旧 u16 格式 (27 字节)，兼容 v1.0 固件
    # - max_steps >  65535 → 新 u32 格式 (29 字节)，需固件同步支持，
    #   否则旧固件会将其截断为低 16 位 (如 1000000 → 16960)
    fmt = "<H I ffff I B H" if max_steps > 65535 else "<H I ffff H B H"
    payload = struct.pack(
        fmt,
        step_pulse,
        step_speed_sps,
        thresh_pre_nA,
        thresh_lock_nA,
        z_start_V,
        z_speed_Vps,
        max_steps,
        retry,
        microstep,
    )
    return _make(Cmd.APPROACH_START, payload)

def approach_stop() -> Frame:
    return _make(Cmd.APPROACH_STOP)

def set_calib(
    tia_R: float,
    x_nmV: float,
    y_nmV: float,
    z_nmV: float,
) -> Frame:
    return _make(Cmd.SET_CALIB, struct.pack("<ffff", tia_R, x_nmV, y_nmV, z_nmV))

def save_calib() -> Frame:
    return _make(Cmd.SAVE_CALIB)

def load_calib() -> Frame:
    return _make(Cmd.LOAD_CALIB)

def get_status() -> Frame:
    return _make(Cmd.GET_STATUS)

def get_fw_version() -> Frame:
    """GET_FW_VERSION: 请求固件版本号"""
    return _make(Cmd.GET_FW_VERSION)


# ---------------------------------------------------------------------------
# 下行帧解析分发
# ---------------------------------------------------------------------------

@dataclass
class ParsedReply:
    """下位机回复的统一包装"""
    kind: Reply
    data: object   # 具体 dataclass 实例 (Status/ScanRow/Ack/Nack/...)

    @classmethod
    def from_frame(cls, frame: Frame) -> "ParsedReply":
        cmd = frame.cmd
        payload = frame.payload
        try:
            reply = Reply(cmd)
        except ValueError:
            raise UnknownCmdError(cmd)
        if reply == Reply.PONG:
            return cls(kind=reply, data=None)
        if reply == Reply.FW_VERSION:
            return cls(kind=reply, data=payload.decode("ascii", errors="replace"))
        if reply == Reply.STATUS:
            return cls(kind=reply, data=Status.unpack(payload))
        if reply == Reply.SCAN_ROW:
            return cls(kind=reply, data=ScanRow.unpack(payload))
        if reply == Reply.SCAN_END:
            return cls(kind=reply, data=ScanEnd.unpack(payload))
        if reply == Reply.APPROACH_DATA:
            return cls(kind=reply, data=ApproachData.unpack(payload))
        if reply == Reply.APPROACH_DONE:
            return cls(kind=reply, data=ApproachDone.unpack(payload))
        if reply == Reply.ERROR:
            return cls(kind=reply, data=ErrorFrame.unpack(payload))
        if reply == Reply.ACK:
            return cls(kind=reply, data=Ack.unpack(payload))
        if reply == Reply.NACK:
            return cls(kind=reply, data=Nack.unpack(payload))
        raise UnknownCmdError(cmd)


# ---------------------------------------------------------------------------
# 自检
# ---------------------------------------------------------------------------

def _self_test() -> None:
    """协议层自检 (可被 pytest 替代)"""
    # CRC 已知向量: "123456789" → 0x4B37
    assert crc16_modbus(b"123456789") == 0x4B37, "CRC-16/MODBUS 自检失败"
    assert crc16_modbus_fast(b"123456789") == 0x4B37, "CRC fast 自检失败"

    # 帧编解码回环
    f = Frame(cmd=0x01, payload=b"")
    raw = f.encode()
    assert raw[:2] == FRAME_HEADER
    assert raw[-1] == FRAME_TAIL
    f2 = Frame.decode(raw)
    assert f2.cmd == f.cmd
    assert f2.payload == f.payload

    # 流式解析器：模拟分片到达
    parser = FrameParser()
    parser.feed(raw[:3])
    assert parser.next_frame() is None
    parser.feed(raw[3:5])
    assert parser.next_frame() is None
    parser.feed(raw[5:])
    frame = parser.next_frame()
    assert frame is not None
    assert frame.cmd == 0x01

    # 构造器 + 解析回环
    f = scan_start(0, 0, 1, 1, 256, 256, 1000)
    raw = f.encode()
    parsed = Frame.decode(raw)
    assert parsed.cmd == int(Cmd.SCAN_START)
    assert len(parsed.payload) == 31

    # STATUS 解析
    payload = struct.pack("<BffffIB", 1, 1.0, 0.5, 0.0, 0.0, 100, 0)
    s = Status.unpack(payload)
    assert s.mode == Mode.FEEDBACK
    assert s.current_nA == 1.0
    assert s.step_pos == 100
    assert s.err == ErrCode.NONE

    # 错误的 CRC 应被捕获
    bad = bytearray(raw)
    bad[5] ^= 0xFF   # 篡改 payload
    parser = FrameParser()
    parser.feed(bytes(bad))
    assert parser.next_frame() is None
    assert parser.proto_err_count[ProtoErr.CRC_MISMATCH] >= 1

    print("protocol.py 自检通过")


if __name__ == "__main__":
    _self_test()