"""
STM 设备控制层

封装 Transport，提供面向 STM 业务的高层 API：
- 标定参数管理 (set/save/load)
- 模式切换
- PID / setpoint / 限幅配置
- 手动定位 (MOVE_XYZ / SET_Z)
- 自动 approach
- 扫描启停 (start/stop/pause/resume)
- 状态查询与回调订阅

所有方法同步阻塞返回结果，异常向上抛出。
UI 层应在 worker 线程调用，避免阻塞主线程。
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

from protocol import (
    ApproachData,
    ApproachDone,
    Cmd,
    DeviceError,
    ErrCode,
    FatalDeviceError,
    Mode,
    NackError,
    Reply,
    ScanDirection,
    ScanEnd,
    ScanMode,
    ScanRow,
    Status,
    approach_start,
    get_fw_version,
    load_calib,
    move_xyz,
    ping,
    retract_z,
    save_calib,
    set_bias,
    scan_pause,
    scan_resume,
    scan_start,
    scan_stop,
    set_calib,
    set_mode,
    set_overcurrent,
    set_pid,
    set_pid_freq,
    set_scan_mode,
    set_setpoint,
    set_z,
    set_z_limits,
    step_motor,
)
from transport import (
    ApproachDataCallback,
    ApproachDoneCallback,
    ErrorCallback,
    NotConnectedError,
    ScanEndCallback,
    ScanRowCallback,
    SendTimeoutError,
    StatusCallback,
    Transport,
    TransportConfig,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 标定数据
# ---------------------------------------------------------------------------

@dataclass
class Calibration:
    """压电标定参数"""
    tia_R: float = 100e6       # TIA 反馈电阻 (Ω)
    x_nmV: float = 0.0         # x 压电灵敏度 (nm/V)
    y_nmV: float = 0.0         # y 压电灵敏度 (nm/V)
    z_nmV: float = 0.0         # z 压电灵敏度 (nm/V)

    def voltage_to_current_nA(self, voltage_V: float) -> float:
        """TIA 输出电压 → 隧道电流 (nA)"""
        return (voltage_V / self.tia_R) * 1e9

    def voltage_to_nm(self, axis: str, voltage_V: float) -> float:
        """DAC 电压 → 位移 (nm)"""
        nmV = {"x": self.x_nmV, "y": self.y_nmV, "z": self.z_nmV}[axis]
        return voltage_V * nmV

    def is_valid(self) -> bool:
        return self.tia_R > 0 and self.x_nmV > 0 and self.y_nmV > 0 and self.z_nmV > 0


# ---------------------------------------------------------------------------
# Approach 参数
# ---------------------------------------------------------------------------

@dataclass
class ApproachParams:
    step_pulse: int = 1
    step_speed_sps: int = 100
    thresh_pre_nA: float = 0.1
    thresh_lock_nA: float = 0.9
    z_start_V: float = 0.0
    z_speed_Vps: float = 1.0
    max_steps: int = 10000
    retry: int = 5
    microstep: int = 16


# ---------------------------------------------------------------------------
# 扫描参数
# ---------------------------------------------------------------------------

@dataclass
class ScanParams:
    cx_V: float = 0.0
    cy_V: float = 0.0
    range_x_V: float = 1.0
    range_y_V: float = 1.0
    nx: int = 256
    ny: int = 256
    speed_pps: int = 1000
    direction: ScanDirection = ScanDirection.BIDIRECTIONAL
    settle_ms: int = 10
    overshoot_V: float = 0.1


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class STMController:
    """
    STM 设备控制器。

    线程安全说明：
    - 内部状态 (mode/status/calib) 由 transport 回调线程更新
    - 控制方法 (set_pid/start_scan 等) 会阻塞等待 ACK
    - UI 层请勿在主线程调用阻塞方法
    """

    def __init__(self, config: Optional[TransportConfig] = None):
        self.transport = Transport(config)
        self._calib = Calibration()
        self._calib_loaded = False
        self._last_status: Optional[Status] = None
        self._state_lock = threading.Lock()

        # 转发 transport 的异步回调给订阅者
        self._on_status: Optional[StatusCallback] = None
        self._on_scan_row: Optional[ScanRowCallback] = None
        self._on_scan_end: Optional[ScanEndCallback] = None
        self._on_approach_data: Optional[ApproachDataCallback] = None
        self._on_approach_done: Optional[ApproachDoneCallback] = None
        self._on_error: Optional[ErrorCallback] = None

        # 绑定 transport 回调
        self.transport.on_status = self._handle_status
        self.transport.on_scan_row = self._handle_scan_row
        self.transport.on_scan_end = self._handle_scan_end
        self.transport.on_approach_data = self._handle_approach_data
        self.transport.on_approach_done = self._handle_approach_done
        self.transport.on_error = self._handle_error

    # -----------------------------------------------------------------
    # 连接管理
    # -----------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.transport.is_connected

    def connect(self, port: Optional[str] = None) -> bool:
        return self.transport.connect(port)

    def disconnect(self) -> None:
        self.transport.disconnect()

    def estop(self) -> None:
        """紧急停止，不抛异常"""
        self.transport.estop()

    # -----------------------------------------------------------------
    # 回调订阅
    # -----------------------------------------------------------------

    def set_callbacks(
        self,
        status: Optional[StatusCallback] = None,
        scan_row: Optional[ScanRowCallback] = None,
        scan_end: Optional[ScanEndCallback] = None,
        approach_data: Optional[ApproachDataCallback] = None,
        approach_done: Optional[ApproachDoneCallback] = None,
        error: Optional[ErrorCallback] = None,
    ) -> None:
        if status is not None: self._on_status = status
        if scan_row is not None: self._on_scan_row = scan_row
        if scan_end is not None: self._on_scan_end = scan_end
        if approach_data is not None: self._on_approach_data = approach_data
        if approach_done is not None: self._on_approach_done = approach_done
        if error is not None: self._on_error = error

    def _handle_status(self, s: Status) -> None:
        with self._state_lock:
            self._last_status = s
        if self._on_status:
            self._on_status(s)

    def _handle_scan_row(self, row: ScanRow) -> None:
        if self._on_scan_row: self._on_scan_row(row)

    def _handle_scan_end(self, end: ScanEnd) -> None:
        if self._on_scan_end: self._on_scan_end(end)

    def _handle_approach_data(self, d: ApproachData) -> None:
        if self._on_approach_data: self._on_approach_data(d)

    def _handle_approach_done(self, d: ApproachDone) -> None:
        if self._on_approach_done: self._on_approach_done(d)

    def _handle_error(self, e: DeviceError) -> None:
        if self._on_error: self._on_error(e)

    # -----------------------------------------------------------------
    # 状态查询
    # -----------------------------------------------------------------

    @property
    def last_status(self) -> Optional[Status]:
        with self._state_lock:
            return self._last_status

    @property
    def current_mode(self) -> Optional[Mode]:
        with self._state_lock:
            return self._last_status.mode if self._last_status else None

    @property
    def is_locked(self) -> bool:
        with self._state_lock:
            return self._last_status.is_locked if self._last_status else False

    def query_status(self) -> Status:
        """主动查询，同步返回"""
        return self.transport.query_status()

    # -----------------------------------------------------------------
    # 标定
    # -----------------------------------------------------------------

    @property
    def calibration(self) -> Calibration:
        return self._calib

    @property
    def calib_loaded(self) -> bool:
        return self._calib_loaded

    def set_calibration(
        self,
        tia_R: float,
        x_nmV: float,
        y_nmV: float,
        z_nmV: float,
        persist: bool = True,
    ) -> None:
        """设置标定参数。persist=True 同时保存到 NVS。"""
        invalid = []
        if tia_R <= 0:
            invalid.append("TIA R (Ω)")
        if x_nmV <= 0:
            invalid.append("x 灵敏度 (nm/V)")
        if y_nmV <= 0:
            invalid.append("y 灵敏度 (nm/V)")
        if z_nmV <= 0:
            invalid.append("z 灵敏度 (nm/V)")
        if invalid:
            raise ValueError(
                f"标定参数必须大于 0，以下字段无效: {', '.join(invalid)}"
            )
        self.transport.send(Cmd.SET_CALIB, set_calib(tia_R, x_nmV, y_nmV, z_nmV).payload)
        self._calib = Calibration(tia_R, x_nmV, y_nmV, z_nmV)
        self._calib_loaded = True
        if persist:
            self.transport.send(Cmd.SAVE_CALIB)

    def save_calibration(self) -> None:
        """将当前标定参数保存到 NVS"""
        self.transport.send(Cmd.SAVE_CALIB)

    def load_calibration(self) -> None:
        """从 NVS 加载标定 (会请求下位机，但不返回具体值)"""
        self.transport.send(Cmd.LOAD_CALIB)
        self._calib_loaded = True

    # -----------------------------------------------------------------
    # 模式切换
    # -----------------------------------------------------------------

    def set_mode(self, mode: Mode) -> None:
        """切换工作模式"""
        self.transport.send(Cmd.SET_MODE, bytes([int(mode)]))

    def enter_feedback(self) -> None:
        self.set_mode(Mode.FEEDBACK)

    def enter_idle(self) -> None:
        self.set_mode(Mode.IDLE)

    def unlock(self) -> None:
        """从 LOCKED 状态解锁 (需先排除硬件故障)"""
        self.set_mode(Mode.IDLE)

    # -----------------------------------------------------------------
    # 反馈参数
    # -----------------------------------------------------------------

    def set_setpoint(self, current_nA: float) -> None:
        self.transport.send(Cmd.SET_SETPOINT, set_setpoint(current_nA).payload)

    def set_pid(self, kp: float, ki: float, kd: float) -> None:
        self.transport.send(Cmd.SET_PID, set_pid(kp, ki, kd).payload)

    def set_pid_freq(self, freq_Hz: int) -> None:
        self.transport.send(Cmd.SET_PID_FREQ, set_pid_freq(freq_Hz).payload)

    def set_z_limits(self, z_min_V: float, z_max_V: float) -> None:
        self.transport.send(Cmd.SET_Z_LIMITS, set_z_limits(z_min_V, z_max_V).payload)

    def set_overcurrent(self, I_limit_nA: float) -> None:
        self.transport.send(Cmd.SET_OVERCURRENT, set_overcurrent(I_limit_nA).payload)

    # -----------------------------------------------------------------
    # 手动定位
    # -----------------------------------------------------------------

    def move_xyz(self, x_V: float, y_V: float, z_V: float) -> None:
        """手动定位 (仅 idle 模式)"""
        self.transport.send(Cmd.MOVE_XYZ, move_xyz(x_V, y_V, z_V).payload)

    def set_z(self, z_V: float) -> None:
        """单独设 z (idle 或 feedback 模式)"""
        self.transport.send(Cmd.SET_Z, set_z(z_V).payload)

    def retract_z(self, steps: int, speed_sps: int = 0) -> None:
        """步进电机后退指定步数 + z DAC 归零

        Args:
            steps: 后退步数 (1~65535)
            speed_sps: 后退速度 (steps/s)，0=用固件默认速度
        """
        frame = retract_z(steps, speed_sps)
        logger.info(
            "RETRACT_Z: steps=%d speed=%d payload=%s (%d bytes)",
            steps, speed_sps, frame.payload.hex(), len(frame.payload)
        )
        self.transport.send(Cmd.RETRACT_Z, frame.payload)

    def set_bias(self, bias_V: float) -> None:
        """设置样品偏压电压"""
        self.transport.send(Cmd.SET_BIAS, set_bias(bias_V).payload)

    def step_motor(self, steps: int, speed_sps: int = 100) -> None:
        """手动步进电机（steps 正=进，负=退）"""
        frame = step_motor(steps, speed_sps)
        logger.info(
            "STEP_MOTOR: steps=%d speed=%d payload=%s (%d bytes)",
            steps, speed_sps, frame.payload.hex(), len(frame.payload)
        )
        self.transport.send(Cmd.STEP_MOTOR, frame.payload)

    def set_scan_mode(self, mode: ScanMode) -> None:
        """设置扫描模式（恒流/恒高）"""
        self.transport.send(Cmd.SET_SCAN_MODE, set_scan_mode(mode).payload)

    # -----------------------------------------------------------------
    # 扫描
    # -----------------------------------------------------------------

    def start_scan(self, params: ScanParams) -> None:
        """启动扫描 (要求当前在 feedback 模式)"""
        frame = scan_start(
            cx_V=params.cx_V,
            cy_V=params.cy_V,
            range_x_V=params.range_x_V,
            range_y_V=params.range_y_V,
            nx=params.nx,
            ny=params.ny,
            speed_pps=params.speed_pps,
            direction=params.direction,
            settle_ms=params.settle_ms,
            overshoot_V=params.overshoot_V,
        )
        self.transport.send(Cmd.SCAN_START, frame.payload)

    def stop_scan(self) -> None:
        self.transport.send(Cmd.SCAN_STOP)

    def pause_scan(self) -> None:
        self.transport.send(Cmd.SCAN_PAUSE)

    def resume_scan(self) -> None:
        self.transport.send(Cmd.SCAN_RESUME)

    # -----------------------------------------------------------------
    # Approach
    # -----------------------------------------------------------------

    def start_approach(self, params: ApproachParams) -> None:
        frame = approach_start(
            step_pulse=params.step_pulse,
            step_speed_sps=params.step_speed_sps,
            thresh_pre_nA=params.thresh_pre_nA,
            thresh_lock_nA=params.thresh_lock_nA,
            z_start_V=params.z_start_V,
            z_speed_Vps=params.z_speed_Vps,
            max_steps=params.max_steps,
            retry=params.retry,
            microstep=params.microstep,
        )
        if params.max_steps > 65535:
            logger.warning(
                "max_steps=%d > 65535, using u32 protocol format; legacy u16 "
                "firmware would truncate it to %d",
                params.max_steps, params.max_steps & 0xFFFF,
            )
        self.transport.send(Cmd.APPROACH_START, frame.payload)

    def stop_approach(self) -> None:
        self.transport.send(Cmd.APPROACH_STOP)

    def get_fw_version(self) -> str | None:
        """获取固件版本号"""
        return self.transport.query(Cmd.GET_FW_VERSION, get_fw_version().payload, Reply.FW_VERSION)

    # -----------------------------------------------------------------
    # 完整工作流：标准 STM 操作序列
    # -----------------------------------------------------------------

    def full_approach_workflow(
        self,
        setpoint_nA: float,
        approach_params: ApproachParams,
        calib: Optional[Calibration] = None,
    ) -> None:
        """
        执行完整的 approach 工作流：
        1. 设置标定 (如提供)
        2. 进入 approach 模式
        3. 设置 setpoint
        4. 启动 approach

        注意：approach 完成后下位机会自动切到 idle，
              上位机需手动调用 enter_feedback() 开启反馈。
        """
        if calib is not None:
            self.set_calibration(
                calib.tia_R, calib.x_nmV, calib.y_nmV, calib.z_nmV
            )
        self.set_mode(Mode.APPROACH)
        self.set_setpoint(setpoint_nA)
        self.start_approach(approach_params)

    def full_scan_workflow(
        self,
        setpoint_nA: float,
        pid: tuple[float, float, float],
        scan: ScanParams,
    ) -> None:
        """
        执行完整的扫描启动流程：
        1. 进入 feedback 模式
        2. 设置 setpoint
        3. 设置 PID
        4. 启动扫描
        """
        self.set_mode(Mode.FEEDBACK)
        self.set_setpoint(setpoint_nA)
        self.set_pid(*pid)
        self.start_scan(scan)


# ---------------------------------------------------------------------------
# 异常再导出 (便于 UI 层统一 import)
# ---------------------------------------------------------------------------

__all__ = [
    "STMController",
    "Calibration",
    "ApproachParams",
    "ScanParams",
    "NotConnectedError",
    "SendTimeoutError",
    "NackError",
    "DeviceError",
    "FatalDeviceError",
    "ErrCode",
    "Mode",
]
