#!/usr/bin/env python3
"""
STM-FORESP32 上位机驱动 (v1.2 协议)

实现:
  - 协议帧收发 (HEADER/LEN/CMD/PAYLOAD/CRC/TAIL)
  - 心跳监控 (PING/PONG, pong_timeout=3s)
  - 断连处理 (停止后台线程, 清理 ACK 队列)
  - 自动重连 (reconnect_interval=1s)
  - 异步命令 (发命令 → 等 ACK, 带超时)
  - 事件回调 (status / scan_row / error / fw_version / connection_state)

用法:
    from STMHost import STMHost

    def on_status(st):
        print(f"mode={st['mode']} I={st['I']:.2f} z={st['z']:.2f}")

    host = STMHost("COM3", on_status=on_status, auto_reconnect=True)
    host.connect()
    host.set_mode(1)          # feedback
    host.set_setpoint(1.0)    # 1 nA
    ...
    host.disconnect()
"""

import struct
import threading
import time
from enum import Enum

try:
    import serial
except ImportError:
    raise ImportError("请先安装 pyserial: pip install pyserial")


# ==================================================================
#  协议常量
# ==================================================================
HEADER = b"\x55\xAA"
TAIL = b"\x0A"

# 上行命令码
CMD_PING = 0x01
CMD_ESTOP = 0x02
CMD_SET_MODE = 0x10
CMD_SET_SETPOINT = 0x11
CMD_SET_PID = 0x12
CMD_SET_Z_LIMITS = 0x13
CMD_SET_PID_FREQ = 0x14
CMD_SET_OVERCURRENT = 0x15
CMD_MOVE_XYZ = 0x20
CMD_SET_Z = 0x21
CMD_RETRACT_Z = 0x22
CMD_SET_BIAS = 0x23
CMD_STEP_MOTOR = 0x24
CMD_SET_SCAN_MODE = 0x25
CMD_SCAN_START = 0x30
CMD_SCAN_STOP = 0x31
CMD_SCAN_PAUSE = 0x32
CMD_SCAN_RESUME = 0x33
CMD_APPROACH_START = 0x40
CMD_APPROACH_STOP = 0x41
CMD_SET_CALIB = 0x50
CMD_SAVE_CALIB = 0x51
CMD_LOAD_CALIB = 0x52
CMD_GET_STATUS = 0x60
CMD_GET_FW_VERSION = 0x61

# 下行命令码
CMD_PONG = 0x81
CMD_FW_VERSION = 0x82
CMD_STATUS = 0x90
CMD_SCAN_ROW = 0x91
CMD_SCAN_END = 0x92
CMD_APPROACH_DATA = 0x93
CMD_APPROACH_DONE = 0x94
CMD_ERROR = 0xA0
CMD_ACK = 0xB0
CMD_NACK = 0xB1


class ConnectionState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"


# ==================================================================
#  CRC-16/MODBUS
# ==================================================================
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    length = 3 + len(payload)
    crc_body = bytes([cmd]) + payload
    crc = crc16_modbus(crc_body)
    return HEADER + struct.pack("<H", length) + crc_body + struct.pack("<H", crc) + TAIL


# ==================================================================
#  帧解析器
# ==================================================================
class FrameReader:
    def __init__(self):
        self.buf = bytearray()
        self.state = "HEAD0"
        self.need = 7

    def feed(self, data: bytes):
        for b in data:
            self.buf.append(b)
            if self.state == "HEAD0":
                if b == 0x55:
                    self.state = "HEAD1"
                else:
                    self.buf.clear()
            elif self.state == "HEAD1":
                if b == 0xAA:
                    self.state = "LEN"
                    self.buf = bytearray(b"\x55\xAA")
                else:
                    self.buf.clear()
                    self.state = "HEAD0"
                    if b == 0x55:
                        self.state = "HEAD1"
                        self.buf.append(0x55)
            elif self.state == "LEN":
                if len(self.buf) == 4:
                    length = struct.unpack("<H", self.buf[2:4])[0]
                    if length < 3 or length > 1027:
                        self.buf.clear()
                        self.state = "HEAD0"
                        continue
                    self.need = 4 + length + 1
                    self.state = "BODY"
            if self.state == "BODY" and len(self.buf) >= self.need:
                frame = self.buf[:self.need]
                self.buf = self.buf[self.need:]
                self.state = "HEAD0"
                if frame[-1:] != TAIL:
                    continue
                length = struct.unpack("<H", frame[2:4])[0]
                cmd = frame[4]
                payload = frame[5:5 + length - 3]
                crc_recv = struct.unpack("<H", frame[5 + length - 3:5 + length - 1])[0]
                crc_calc = crc16_modbus(frame[4:5 + length - 3])
                if crc_recv != crc_calc:
                    continue
                yield (cmd, payload)


# ==================================================================
#  STMHost 驱动
# ==================================================================
class STMHost:
    """
    上位机驱动: 线程安全, 后台读/心跳/重连线程。

    参数:
        port          串口设备
        baud          波特率
        on_status     STATUS 回调 (dict)
        on_scan_row   SCAN_ROW 回调 (dict)
        on_error      ERROR 回调 (dict)
        on_fw_version FW_VERSION 回调 (str)
        on_state      连接状态回调 (ConnectionState)
        auto_reconnect 断连后自动重连
        pong_timeout  PONG 超时阈值 (s), 默认 3
        ping_interval PING 发送间隔 (s), 默认 1
        reconnect_interval 重连间隔 (s), 默认 1
        ack_timeout   命令 ACK 超时 (s), 默认 2
    """

    def __init__(self, port, baud=115200,
                 on_status=None, on_scan_row=None, on_error=None,
                 on_fw_version=None, on_state=None,
                 auto_reconnect=True,
                 pong_timeout=3.0, ping_interval=1.0,
                 reconnect_interval=1.0, ack_timeout=2.0):
        self.port = port
        self.baud = baud
        self._cb_status = on_status
        self._cb_scan_row = on_scan_row
        self._cb_error = on_error
        self._cb_fw_version = on_fw_version
        self._cb_state = on_state

        self.auto_reconnect = auto_reconnect
        self.pong_timeout = pong_timeout
        self.ping_interval = ping_interval
        self.reconnect_interval = reconnect_interval
        self.ack_timeout = ack_timeout

        self._ser = None
        self._reader = FrameReader()
        self._state = ConnectionState.DISCONNECTED
        self._lock = threading.Lock()

        # 后台线程
        self._thread_read = None
        self._thread_hb = None
        self._stop_event = threading.Event()

        # ACK 等待队列: { cmd_code: [Event, result_dict_or_None] }
        self._ack_conds = {}
        self._ack_lock = threading.Lock()

        # 心跳
        self._last_pong_ts = 0.0
        self._last_ping_ts = 0.0

        # 用户主动断开标志 (区别于异常断连)
        self._user_disconnect = False

    # ---------- 公共属性 ----------
    @property
    def state(self) -> ConnectionState:
        return self._state

    @property
    def is_connected(self) -> bool:
        return self._state == ConnectionState.CONNECTED

    # ---------- 连接管理 ----------
    def connect(self, timeout=5.0):
        """主动连接 (阻塞, 直到成功或超时)"""
        self._user_disconnect = False
        if not self._open_serial():
            return False
        self._start_threads()
        # 等待首个 PONG 确认链路
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.is_connected:
                return True
            time.sleep(0.05)
        return self.is_connected

    def disconnect(self):
        """主动断开 (不触发自动重连)"""
        self._user_disconnect = True
        self._handle_disconnect(reason="user_disconnect")

    def _open_serial(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self._set_state(ConnectionState.CONNECTING)
            return True
        except Exception:
            self._ser = None
            return False

    def _close_serial(self):
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _start_threads(self):
        self._stop_event.clear()
        self._thread_read = threading.Thread(target=self._read_loop, daemon=True)
        self._thread_hb = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._thread_read.start()
        self._thread_hb.start()

    def _set_state(self, st: ConnectionState):
        if self._state != st:
            self._state = st
            if self._cb_state:
                try:
                    self._cb_state(st)
                except Exception:
                    pass

    # ---------- 后台读线程 ----------
    def _read_loop(self):
        while not self._stop_event.is_set():
            if self._ser is None:
                time.sleep(0.05)
                continue
            try:
                data = self._ser.read(256)
            except Exception:
                self._handle_disconnect(reason="read_error")
                return
            if data:
                for cmd, payload in self._reader.feed(data):
                    self._dispatch_frame(cmd, payload)

    def _dispatch_frame(self, cmd: int, payload: bytes):
        if cmd == CMD_PONG:
            self._last_pong_ts = time.time()
            self._set_state(ConnectionState.CONNECTED)
            return
        if cmd == CMD_FW_VERSION:
            ver = payload.decode("ascii", errors="replace")
            if self._cb_fw_version:
                try:
                    self._cb_fw_version(ver)
                except Exception:
                    pass
            return
        if cmd == CMD_ACK:
            self._notify_ack(payload[0], ok=True, reason=0)
            return
        if cmd == CMD_NACK:
            self._notify_ack(payload[0], ok=False, reason=payload[1])
            return
        if cmd == CMD_STATUS:
            st = self._decode_status(payload)
            if st and self._cb_status:
                try:
                    self._cb_status(st)
                except Exception:
                    pass
            return
        if cmd == CMD_SCAN_ROW:
            row = self._decode_scan_row(payload)
            if row and self._cb_scan_row:
                try:
                    self._cb_scan_row(row)
                except Exception:
                    pass
            return
        if cmd == CMD_ERROR:
            err = self._decode_error(payload)
            if err and self._cb_error:
                try:
                    self._cb_error(err)
                except Exception:
                    pass
            return
        # SCAN_END / APPROACH_DATA / APPROACH_DONE 可扩展

    # ---------- 后台心跳线程 ----------
    def _heartbeat_loop(self):
        while not self._stop_event.is_set():
            now = time.time()
            if self._ser is None:
                time.sleep(0.05)
                continue
            # 定时发 PING
            if now - self._last_ping_ts >= self.ping_interval:
                self._last_ping_ts = now
                try:
                    self._ser.write(build_frame(CMD_PING))
                except Exception:
                    self._handle_disconnect(reason="write_error")
                    return
            # 检查 PONG 超时
            if self._last_pong_ts > 0 and (now - self._last_pong_ts) > self.pong_timeout:
                self._handle_disconnect(reason="pong_timeout")
                return
            time.sleep(0.05)

    # ---------- 断连处理 ----------
    def _handle_disconnect(self, reason="unknown"):
        """停止线程 + 清理队列 + 状态回调 (+ 可选自动重连)"""
        if self._state == ConnectionState.DISCONNECTED:
            return  # 已处理
        self._stop_event.set()
        self._close_serial()
        # 清理所有等待 ACK 的命令, 防止死锁
        with self._ack_lock:
            for cond in self._ack_conds.values():
                ev = cond[0]
                cond[1] = {"ok": False, "reason": 0xFF, "disconnected": True}
                ev.set()
            self._ack_conds.clear()
        self._set_state(ConnectionState.DISCONNECTED)

        # 用户主动断开: 不重连
        if self._user_disconnect:
            return

        # 自动重连
        if self.auto_reconnect:
            self._reconnect_loop()

    def _reconnect_loop(self):
        self._set_state(ConnectionState.RECONNECTING)
        while not self._user_disconnect:
            time.sleep(self.reconnect_interval)
            if self._open_serial():
                self._reader = FrameReader()
                self._last_pong_ts = 0.0
                self._last_ping_ts = 0.0
                self._stop_event.clear()
                self._start_threads()
                return  # 重连成功, 交回读/心跳线程
            # 继续尝试
            self._set_state(ConnectionState.RECONNECTING)

    # ---------- ACK 同步机制 ----------
    def _notify_ack(self, cmd_code: int, ok: bool, reason: int):
        with self._ack_lock:
            cond = self._ack_conds.get(cmd_code)
            if cond:
                cond[1] = {"ok": ok, "reason": reason, "disconnected": False}
                cond[0].set()

    def _send_and_wait(self, cmd: int, payload: bytes = b"") -> dict:
        """发送命令并等待 ACK/NACK, 返回 {"ok":bool, "reason":int}"""
        if not self.is_connected and self._ser is None:
            return {"ok": False, "reason": 0xFE, "disconnected": True}
        # 注册等待
        ev = threading.Event()
        slot = [ev, None]
        with self._ack_lock:
            self._ack_conds[cmd] = slot
        # 发送
        try:
            frame = build_frame(cmd, payload)
            if self._ser:
                self._ser.write(frame)
        except Exception:
            with self._ack_lock:
                self._ack_conds.pop(cmd, None)
            return {"ok": False, "reason": 0xFD, "disconnected": True}
        # 等待
        got = ev.wait(timeout=self.ack_timeout)
        with self._ack_lock:
            self._ack_conds.pop(cmd, None)
        if not got:
            return {"ok": False, "reason": 0xFC, "disconnected": False}  # timeout
        return slot[1]

    # ---------- 高层命令 API ----------
    def ping(self):
        """发 PING (异步, 不等待 ACK; PONG 由心跳线程处理)"""
        if self._ser:
            try:
                self._ser.write(build_frame(CMD_PING))
                self._last_ping_ts = time.time()
                return True
            except Exception:
                return False
        return False

    def estop(self) -> dict:
        return self._send_and_wait(CMD_ESTOP)

    def set_mode(self, mode: int) -> dict:
        return self._send_and_wait(CMD_SET_MODE, bytes([mode]))

    def set_setpoint(self, I_nA: float) -> dict:
        return self._send_and_wait(CMD_SET_SETPOINT, struct.pack("<f", I_nA))

    def set_pid(self, Kp: float, Ki: float, Kd: float) -> dict:
        return self._send_and_wait(CMD_SET_PID, struct.pack("<fff", Kp, Ki, Kd))

    def set_z_limits(self, z_min: float, z_max: float) -> dict:
        return self._send_and_wait(CMD_SET_Z_LIMITS, struct.pack("<ff", z_min, z_max))

    def set_pid_freq(self, freq_hz: int) -> dict:
        return self._send_and_wait(CMD_SET_PID_FREQ, struct.pack("<I", freq_hz))

    def set_overcurrent(self, I_nA: float) -> dict:
        return self._send_and_wait(CMD_SET_OVERCURRENT, struct.pack("<f", I_nA))

    def move_xyz(self, x: float, y: float, z: float) -> dict:
        return self._send_and_wait(CMD_MOVE_XYZ, struct.pack("<fff", x, y, z))

    def set_z(self, z: float) -> dict:
        return self._send_and_wait(CMD_SET_Z, struct.pack("<f", z))

    def retract_z(self, steps: int, speed: int = 0) -> dict:
        """RETRACT_Z (v1.2): 步进后退 + z DAC 全退
        steps: 后退步数
        speed: 后退速度 (steps/s), 0=用固件默认速度"""
        return self._send_and_wait(CMD_RETRACT_Z, struct.pack("<HI", steps, speed))

    def set_bias(self, bias_V: float) -> dict:
        return self._send_and_wait(CMD_SET_BIAS, struct.pack("<f", bias_V))

    def step_motor(self, steps: int, direction: int, speed_sps: int) -> dict:
        return self._send_and_wait(CMD_STEP_MOTOR, struct.pack("<HBI", steps, direction, speed_sps))

    def set_scan_mode(self, mode: int) -> dict:
        return self._send_and_wait(CMD_SET_SCAN_MODE, bytes([mode]))

    def scan_start(self, cx, cy, rx, ry, nx, ny, speed, direction, settle_ms, overshoot_V) -> dict:
        payload = struct.pack("<ffffHHIBHf", cx, cy, rx, ry, nx, ny, speed, direction, settle_ms, overshoot_V)
        return self._send_and_wait(CMD_SCAN_START, payload)

    def scan_stop(self) -> dict:
        return self._send_and_wait(CMD_SCAN_STOP)

    def scan_pause(self) -> dict:
        return self._send_and_wait(CMD_SCAN_PAUSE)

    def scan_resume(self) -> dict:
        return self._send_and_wait(CMD_SCAN_RESUME)

    def approach_start(self, step_pulse, step_sps, thresh_pre, thresh_lock,
                       z_start, z_speed, max_steps, retry, microstep) -> dict:
        payload = struct.pack("<HIffffH BH", step_pulse, step_sps, thresh_pre, thresh_lock,
                              z_start, z_speed, max_steps, retry, microstep)
        return self._send_and_wait(CMD_APPROACH_START, payload)

    def approach_stop(self) -> dict:
        return self._send_and_wait(CMD_APPROACH_STOP)

    def set_calib(self, tia_R, x_nmV, y_nmV, z_nmV) -> dict:
        return self._send_and_wait(CMD_SET_CALIB, struct.pack("<ffff", tia_R, x_nmV, y_nmV, z_nmV))

    def save_calib(self) -> dict:
        return self._send_and_wait(CMD_SAVE_CALIB)

    def load_calib(self) -> dict:
        return self._send_and_wait(CMD_LOAD_CALIB)

    def get_status(self):
        """主动查询状态 (下位机会立即回 STATUS 帧, 由 on_status 回调接收)"""
        if self._ser:
            try:
                self._ser.write(build_frame(CMD_GET_STATUS))
                return True
            except Exception:
                return False
        return False

    def get_fw_version(self):
        """主动查询固件版本号 (下位机回 FW_VERSION 帧, 由 on_fw_version 回调接收)"""
        if self._ser:
            try:
                self._ser.write(build_frame(CMD_GET_FW_VERSION))
                return True
            except Exception:
                return False
        return False

    # ---------- 帧解码 ----------
    @staticmethod
    def _decode_status(payload: bytes):
        if len(payload) < 22:
            return None
        mode, I, z, x, y = struct.unpack("<Bffff", payload[:17])
        step_pos = struct.unpack("<i", payload[17:21])[0]
        err = payload[21]
        names = {0: "IDLE", 1: "FEEDBACK", 2: "SCAN", 3: "APPROACH"}
        return {"mode": names.get(mode, str(mode)), "mode_id": mode,
                "I": I, "z": z, "x": x, "y": y, "step_pos": step_pos, "err": err}

    @staticmethod
    def _decode_scan_row(payload: bytes):
        if len(payload) < 8:
            return None
        y_idx, nx = struct.unpack("<HH", payload[:4])
        n = nx
        if len(payload) < 4 + 8 * n:
            return None
        I_data = struct.unpack(f"<{n}f", payload[4:4 + 4 * n])
        z_data = struct.unpack(f"<{n}f", payload[4 + 4 * n:4 + 8 * n])
        return {"y_idx": y_idx, "nx": nx, "I": I_data, "z": z_data}

    @staticmethod
    def _decode_error(payload: bytes):
        if len(payload) < 2:
            return None
        code = payload[0]
        msg_len = payload[1]
        msg = payload[2:2 + msg_len].decode("ascii", errors="replace")
        return {"code": code, "msg": msg}