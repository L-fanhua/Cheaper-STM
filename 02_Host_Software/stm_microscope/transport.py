"""
STM 串口通信层

负责：
- USB CDC 串口的连接 / 断开 / 自动重连
- 心跳 (PING/PONG) 周期发送与超时检测
- 帧的发送与异步接收 (后台线程)
- 回复与命令的匹配 (基于 ACK/NACK 或特定 reply)
- 错误事件回调通知

依赖：pyserial
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, Union

import serial
import serial.tools.list_ports

from protocol import (
    Ack,
    ApproachData,
    ApproachDone,
    Cmd,
    DeviceError,
    ErrorFrame,
    FatalDeviceError,
    Frame,
    FrameParser,
    Mode,
    Nack,
    NackError,
    ParsedReply,
    ProtoErr,
    Reply,
    ScanEnd,
    ScanRow,
    Status,
    estop as cmd_estop,
    get_status as cmd_get_status,
    ping as cmd_ping,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 事件回调类型
# ---------------------------------------------------------------------------

# 通用事件回调：参数为可读字符串
LogCallback = Callable[[str], None]
# 连接状态变化：connected / disconnected / reconnecting
ConnectionCallback = Callable[[str], None]
# 周期 STATUS 回调
StatusCallback = Callable[[Status], None]
# 扫描数据回调
ScanRowCallback = Callable[[ScanRow], None]
ScanEndCallback = Callable[[ScanEnd], None]
# Approach 过程回调
ApproachDataCallback = Callable[[ApproachData], None]
ApproachDoneCallback = Callable[[ApproachDone], None]
# 错误回调
ErrorCallback = Callable[[DeviceError], None]
# 协议层错误回调 (用于诊断)
ProtoErrorCallback = Callable[[ProtoErr, str], None]
# 原始帧日志回调：(方向, 命令名, payload_hex)
FrameLogCallback = Callable[[str, str, str], None]


# ---------------------------------------------------------------------------
# 连接配置
# ---------------------------------------------------------------------------

@dataclass
class TransportConfig:
    port: str = ""                # 为空则自动检测
    baudrate: int = 115200        # USB CDC 实际不受此限制，但需要给个值
    bytesize: int = 8
    parity: str = "N"
    stopbits: float = 1
    timeout: float = 0.1          # 单次 read 超时 (秒)
    ping_interval: float = 1.0    # PING 发送间隔
    pong_timeout: float = 3.0     # PONG 超时 → 判定断连
    auto_reconnect: bool = True
    reconnect_interval: float = 1.0  # 重连尝试间隔
    send_timeout: float = 2.0     # 命令发送等待 ACK 超时
    # ESP32-S3 USB CDC 通常的 VID:PID
    usb_vid: int = 0x303A         # Espressif
    usb_pid: int = None           # 任意


# ---------------------------------------------------------------------------
# 命令未完成异常
# ---------------------------------------------------------------------------

class SendTimeoutError(Exception):
    """命令发送后未在超时内收到 ACK/NACK/期望回复"""
    def __init__(self, cmd: Cmd, timeout: float):
        self.cmd = cmd
        self.timeout = timeout
        super().__init__(f"Send timeout for {cmd.name} after {timeout}s")


class NotConnectedError(Exception):
    """未连接时尝试发送命令"""


# ---------------------------------------------------------------------------
# Transport 主体
# ---------------------------------------------------------------------------

class Transport:
    """
    串口通信层。

    线程模型：
    - 主线程：调用 send_xxx() 发命令
    - 后台读线程：读串口 → 解析帧 → 分发回调 / 唤醒等待者
    - 后台心跳线程：周期发 PING
    """

    def __init__(self, config: Optional[TransportConfig] = None):
        self.config = config or TransportConfig()
        self._serial: Optional[serial.Serial] = None
        self._parser = FrameParser()
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._connected = threading.Event()

        # 读线程
        self._read_thread: Optional[threading.Thread] = None
        # 心跳线程
        self._heartbeat_thread: Optional[threading.Thread] = None

        # 命令-回复匹配：cmd_byte → (Event, [result])
        # 当等待 ACK/NACK 时用 cmd_byte 作 key
        self._pending_ack: dict[int, list] = {}
        # 当等待特定 reply 类型时用 reply_byte 作 key
        self._pending_reply: dict[int, list] = {}
        self._pending_lock = threading.Lock()

        # 回调
        self.on_status: Optional[StatusCallback] = None
        self.on_scan_row: Optional[ScanRowCallback] = None
        self.on_scan_end: Optional[ScanEndCallback] = None
        self.on_approach_data: Optional[ApproachDataCallback] = None
        self.on_approach_done: Optional[ApproachDoneCallback] = None
        self.on_error: Optional[ErrorCallback] = None
        self.on_proto_error: Optional[ProtoErrorCallback] = None
        self.on_connection: Optional[ConnectionCallback] = None
        self.on_log: Optional[LogCallback] = None
        self.on_frame_log: Optional[FrameLogCallback] = None  # TX/RX 原始帧日志

        # 心跳状态
        self._last_pong_time: float = 0.0
        self._last_ping_time: float = 0.0

        # 重连标志（由 _handle_disconnect 设置，由外部 UI 调用 start_auto_reconnect 触发）
        self._needs_reconnect: bool = False

    # -----------------------------------------------------------------
    # 串口检测与连接
    # -----------------------------------------------------------------

    @staticmethod
    def list_ports() -> list[str]:
        """列出所有可用串口设备名"""
        return [p.device for p in serial.tools.list_ports.comports()]

    @staticmethod
    def auto_detect(vid: int = 0x303A, pid: Optional[int] = None) -> Optional[str]:
        """自动检测 ESP32-S3 USB CDC 设备"""
        for p in serial.tools.list_ports.comports():
            if p.vid == vid:
                if pid is None or p.pid == pid:
                    return p.device
        return None

    def connect(self, port: Optional[str] = None) -> bool:
        """
        连接设备。
        - port 为空则用 config.port，再为空则自动检测
        - 成功连接后启动后台线程
        """
        if self._connected.is_set():
            self._log("already connected")
            return True

        target = port or self.config.port
        if not target:
            target = self.auto_detect(self.config.usb_vid, self.config.usb_pid)
        if not target:
            self._log("no device found")
            return False

        try:
            self._serial = serial.Serial(
                port=target,
                baudrate=self.config.baudrate,
                bytesize=self.config.bytesize,
                parity=self.config.parity,
                stopbits=self.config.stopbits,
                timeout=self.config.timeout,
            )
        except serial.SerialException as e:
            self._log(f"open {target} failed: {e}")
            return False

        self._log(f"connected to {target}")
        self._running.set()
        self._connected.set()
        self._last_pong_time = time.time()
        self._needs_reconnect = False

        # 启动后台线程
        self._read_thread = threading.Thread(
            target=self._read_loop, name="stm-read", daemon=True
        )
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="stm-heartbeat", daemon=True
        )
        self._read_thread.start()
        self._heartbeat_thread.start()

        self._notify_connection(f"connected:{target}")
        return True

    def disconnect(self) -> None:
        """断开连接"""
        self._running.clear()
        self._connected.clear()

        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)

        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        # 唤醒所有等待者
        with self._pending_lock:
            for _, holder in self._pending_ack.items():
                holder[0].set()
            for _, holder in self._pending_reply.items():
                holder[0].set()
            self._pending_ack.clear()
            self._pending_reply.clear()

        self._notify_connection("disconnected")
        self._log("disconnected")

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    # -----------------------------------------------------------------
    # 后台读循环
    # -----------------------------------------------------------------

    def _read_loop(self) -> None:
        """后台读线程主循环"""
        while self._running.is_set():
            if not self._connected.is_set():
                break
            try:
                if not self._serial or not self._serial.is_open:
                    self._handle_disconnect("serial closed")
                    break
                # 非阻塞读 (timeout 已设为 0.1s)
                data = self._serial.read(1024)
                if data:
                    self._parser.feed(data)
                    self._drain_frames()
            except serial.SerialException as e:
                self._handle_disconnect(f"serial error: {e}")
                break
            except Exception as e:
                logger.exception("read loop error")
                self._log(f"read loop error: {e}")

    def _drain_frames(self) -> None:
        """从 parser 取出所有完整帧并分发"""
        while True:
            try:
                frame = self._parser.next_frame()
            except Exception as e:
                self._log(f"parse error: {e}")
                break
            if frame is None:
                break
            self._dispatch(frame)

    def _dispatch(self, frame: Frame) -> None:
        """分发一帧到对应回调或等待者"""
        try:
            reply = ParsedReply.from_frame(frame)
        except Exception as e:
            self._log(f"dispatch error: {e}")
            return

        kind = reply.kind
        data = reply.data

        # RX 原始帧日志（过滤掉高频的 PONG 和 STATUS，避免刷屏）
        if self.on_frame_log and kind not in (Reply.PONG, Reply.STATUS, Reply.SCAN_ROW):
            try:
                name = self._cmd_name(int(kind))
                # 对 ACK/NACK 追加被确认的命令信息
                if kind == Reply.ACK and isinstance(data, Ack):
                    ack_name = self._cmd_name(data.cmd)
                    self.on_frame_log("RX", f"ACK({ack_name})", "")
                elif kind == Reply.NACK and isinstance(data, Nack):
                    nack_name = self._cmd_name(data.cmd)
                    self.on_frame_log("RX", f"NACK({nack_name})", f"reason={data.reason.name}")
                elif kind == Reply.ERROR and isinstance(data, ErrorFrame):
                    self.on_frame_log("RX", f"ERROR({data.code.name})", data.msg)
                else:
                    self.on_frame_log("RX", name, "")
            except Exception:
                pass

        # 心跳响应
        if kind == Reply.PONG:
            self._last_pong_time = time.time()
            self._wake_reply(int(Reply.PONG), data)
            return

        # FW_VERSION 回复
        if kind == Reply.FW_VERSION:
            self._wake_reply(int(Reply.FW_VERSION), data)
            return
        # ACK / NACK：唤醒等待该 cmd 的发送者
        if kind == Reply.ACK:
            assert isinstance(data, Ack)
            self._wake_ack(data.cmd, success=True)
            return
        if kind == Reply.NACK:
            assert isinstance(data, Nack)
            self._wake_ack(data.cmd, success=False, nack=data)
            return

        # 异步事件：先唤醒等待特定 reply 的，再触发回调
        if kind == Reply.STATUS:
            assert isinstance(data, Status)
            self._wake_reply(int(Reply.STATUS), data)
            if self.on_status:
                self._safe_callback(self.on_status, data)
            return
        if kind == Reply.SCAN_ROW:
            assert isinstance(data, ScanRow)
            self._wake_reply(int(Reply.SCAN_ROW), data)
            if self.on_scan_row:
                self._safe_callback(self.on_scan_row, data)
            return
        if kind == Reply.SCAN_END:
            assert isinstance(data, ScanEnd)
            self._wake_reply(int(Reply.SCAN_END), data)
            if self.on_scan_end:
                self._safe_callback(self.on_scan_end, data)
            return
        if kind == Reply.APPROACH_DATA:
            assert isinstance(data, ApproachData)
            self._wake_reply(int(Reply.APPROACH_DATA), data)
            if self.on_approach_data:
                self._safe_callback(self.on_approach_data, data)
            return
        if kind == Reply.APPROACH_DONE:
            assert isinstance(data, ApproachDone)
            self._wake_reply(int(Reply.APPROACH_DONE), data)
            if self.on_approach_done:
                self._safe_callback(self.on_approach_done, data)
            return

        # ERROR 帧
        if kind == Reply.ERROR:
            assert isinstance(data, ErrorFrame)
            exc = data.to_exception()
            self._wake_reply(int(Reply.ERROR), exc)
            if self.on_error:
                self._safe_callback(self.on_error, exc)
            return

    def _safe_callback(self, cb, *args) -> None:
        """安全调用回调，捕获异常"""
        try:
            cb(*args)
        except Exception as e:
            logger.exception("callback error")
            self._log(f"callback error: {e}")

    # -----------------------------------------------------------------
    # 命令-回复匹配
    # -----------------------------------------------------------------

    def _wake_ack(self, cmd_byte: int, success: bool, nack: Optional[Nack] = None) -> None:
        with self._pending_lock:
            holder = self._pending_ack.pop(cmd_byte, None)
        if holder:
            event, result = holder
            result.append(success if success else (nack or False))
            event.set()

    def _wake_reply(self, reply_byte: int, data) -> None:
        with self._pending_lock:
            holder = self._pending_reply.pop(reply_byte, None)
        if holder:
            event, result = holder
            result.append(data)
            event.set()

    def _wait_ack(self, cmd: Cmd, timeout: float) -> bool:
        """发送后等待 ACK/NACK"""
        event = threading.Event()
        result: list = []
        with self._pending_lock:
            self._pending_ack[int(cmd)] = (event, result)
        if not event.wait(timeout):
            with self._pending_lock:
                self._pending_ack.pop(int(cmd), None)
            raise SendTimeoutError(cmd, timeout)
        if not result:
            raise SendTimeoutError(cmd, timeout)
        val = result[0]
        if val is True:
            return True
        if isinstance(val, Nack):
            raise NackError(int(cmd), val.reason)
        raise SendTimeoutError(cmd, timeout)

    def _wait_reply(self, reply: Reply, timeout: float):
        """等待一个特定 reply (用于 GET_STATUS → STATUS)"""
        event = threading.Event()
        result: list = []
        with self._pending_lock:
            self._pending_reply[int(reply)] = (event, result)
        if not event.wait(timeout):
            with self._pending_lock:
                self._pending_reply.pop(int(reply), None)
            raise SendTimeoutError(Cmd.GET_STATUS, timeout)
        if not result:
            raise SendTimeoutError(Cmd.GET_STATUS, timeout)
        return result[0]

    # -----------------------------------------------------------------
    # 发送
    # -----------------------------------------------------------------

    def _raw_send(self, frame: Frame) -> None:
        """直接发送一帧 (不加锁等待 ACK)"""
        if not self._connected.is_set() or not self._serial:
            raise NotConnectedError()
        # TX 日志（过滤掉高频的 PING，避免刷屏）
        if self.on_frame_log and frame.cmd != int(Cmd.PING):
            try:
                name = self._cmd_name(frame.cmd)
                self.on_frame_log("TX", name, frame.payload.hex())
            except Exception:
                pass
        data = frame.encode()
        with self._lock:
            self._serial.write(data)
            self._serial.flush()

    @staticmethod
    def _cmd_name(cmd_byte: int) -> str:
        """命令码转可读名称"""
        try:
            return Cmd(cmd_byte).name
        except ValueError:
            try:
                return Reply(cmd_byte).name
            except ValueError:
                return f"0x{cmd_byte:02X}"

    def send(self, cmd: Cmd, payload: bytes = b"", *, wait_ack: bool = True,
             timeout: Optional[float] = None) -> bool:
        """
        发送一条命令并 (默认) 等待 ACK。

        - wait_ack=True：阻塞直到收到 ACK，NACK 抛 NackError，超时抛 SendTimeoutError
        - wait_ack=False：只发不等
        """
        frame = Frame(cmd=int(cmd), payload=payload)
        self._raw_send(frame)
        if not wait_ack:
            return True
        return self._wait_ack(cmd, timeout or self.config.send_timeout)

    def query(self, cmd: Cmd, payload: bytes, reply: Reply, timeout: Optional[float] = None):
        """发送命令并等待特定回复，返回回复数据"""
        event = threading.Event()
        result: list = []
        with self._pending_lock:
            self._pending_reply[int(reply)] = (event, result)
        try:
            frame = Frame(cmd=int(cmd), payload=payload)
            self._raw_send(frame)
        except Exception:
            with self._pending_lock:
                self._pending_reply.pop(int(reply), None)
            raise
        timeout_val = timeout or self.config.send_timeout
        if not event.wait(timeout_val):
            with self._pending_lock:
                self._pending_reply.pop(int(reply), None)
            raise SendTimeoutError(cmd, timeout_val)
        if not result:
            raise SendTimeoutError(cmd, timeout_val)
        val = result[0]
        if isinstance(val, Exception):
            raise val
        return val

    # -----------------------------------------------------------------
    # 高层便捷 API
    # -----------------------------------------------------------------

    def ping(self) -> bool:
        """发送 PING 并等待 PONG"""
        if not self._connected.is_set():
            return False
        try:
            self._raw_send(cmd_ping())
            # 心跳线程也会发 PING，这里不阻塞等 PONG，只看连接状态
            return True
        except Exception as e:
            self._log(f"ping send failed: {e}")
            return False

    def estop(self) -> None:
        """紧急停止 - 不等 ACK，最高优先级"""
        try:
            # ESTOP 即使断连也尝试发一次
            if self._serial and self._serial.is_open:
                self._raw_send(cmd_estop())
                self._log("ESTOP sent")
        except Exception as e:
            self._log(f"ESTOP send failed: {e}")

    def query_status(self, timeout: float = 1.0) -> Status:
        """主动查询状态，同步返回"""
        # 注册等待 STATUS
        event = threading.Event()
        result: list = []
        with self._pending_lock:
            self._pending_reply[int(Reply.STATUS)] = (event, result)
        self._raw_send(cmd_get_status())
        if not event.wait(timeout):
            with self._pending_lock:
                self._pending_reply.pop(int(Reply.STATUS), None)
            raise SendTimeoutError(Cmd.GET_STATUS, timeout)
        if not result:
            raise SendTimeoutError(Cmd.GET_STATUS, timeout)
        val = result[0]
        if isinstance(val, Exception):
            raise val
        return val

    # -----------------------------------------------------------------
    # 心跳循环
    # -----------------------------------------------------------------

    def _heartbeat_loop(self) -> None:
        """后台心跳线程"""
        while self._running.is_set():
            if not self._connected.is_set():
                break
            now = time.time()
            # 周期发 PING
            if now - self._last_ping_time >= self.config.ping_interval:
                self._last_ping_time = now
                try:
                    self._raw_send(cmd_ping())
                except Exception as e:
                    self._log(f"heartbeat send failed: {e}")
                    self._handle_disconnect(f"heartbeat failed: {e}")
                    break
            # 检测 PONG 超时
            if now - self._last_pong_time > self.config.pong_timeout:
                self._handle_disconnect("pong timeout")
                break
            # 间隔
            time.sleep(0.2)

    # -----------------------------------------------------------------
    # 断连与重连
    # -----------------------------------------------------------------

    def _handle_disconnect(self, reason: str) -> None:
        """处理断连事件"""
        if not self._connected.is_set():
            return   # 已经处理过
        self._running.clear()  # 确保所有后台线程停止
        self._log(f"disconnect detected: {reason}")
        self._connected.clear()
        self._notify_connection(f"disconnected:{reason}")

        # 关闭串口
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        # 通知错误回调
        if self.on_error:
            from protocol import ErrCode
            err = DeviceError(ErrCode.COMM_TIMEOUT, reason)
            self._safe_callback(self.on_error, err)

        # 标记需要重连，但不自动执行
        self._needs_reconnect = self.config.auto_reconnect

    def start_auto_reconnect(self) -> None:
        """启动自动重连（在断连后由 UI 调用）"""
        if self._needs_reconnect and not self._connected.is_set():
            self._log("starting auto-reconnect loop")
            threading.Thread(
                target=self._reconnect_loop, name="stm-reconnect", daemon=True
            ).start()

    def _reconnect_loop(self) -> None:
        """自动重连循环"""
        while self._needs_reconnect and not self._connected.is_set():
            self._notify_connection("reconnecting")
            if self.connect():
                return
            time.sleep(self.config.reconnect_interval)
        self._needs_reconnect = False

    # -----------------------------------------------------------------
    # 日志与回调通知
    # -----------------------------------------------------------------

    def _log(self, msg: str) -> None:
        logger.info(msg)
        if self.on_log:
            self._safe_callback(self.on_log, msg)

    def _notify_connection(self, state: str) -> None:
        if self.on_connection:
            self._safe_callback(self.on_connection, state)

    # -----------------------------------------------------------------
    # 诊断
    # -----------------------------------------------------------------

    def proto_error_stats(self) -> dict:
        """返回协议层错误统计 (用于诊断固件问题)"""
        return {k.name: v for k, v in self._parser.proto_err_count.items()}

    def reset_parser(self) -> None:
        """重置解析器状态 (清除残留 buffer)"""
        self._parser.reset()