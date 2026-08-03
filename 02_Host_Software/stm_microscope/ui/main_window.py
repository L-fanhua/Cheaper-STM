"""
STM 上位机主窗口

三栏专业式布局：
  ┌─────────────────────────────────────────────────────┐
  │ 顶部状态条 (永驻)                                    │
  ├──────────┬──────────────────────┬──────────────────┤
  │ 左栏      │ 中栏                  │ 右栏              │
  │ 参数折叠组 │ 主图像 + 进度          │ 扫描行示波器      │
  │ (滚动)    │                      │ 日志              │
  │          │                      │                  │
  └──────────┴──────────────────────┴──────────────────┘
"""
from __future__ import annotations

import logging
from typing import Optional

import concurrent.futures
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from controller import STMController
from protocol import DeviceError, ErrCode, FatalDeviceError, Mode, Status
from scanner import ScanEngine, save_scan
from transport import TransportConfig
from ui.widgets import (
    CollapsibleSection,
    ConnectionPanel,
    FeedbackFields,
    ImagePanel,
    LineOscilloscopePanel,
    LogPanel,
    ManualControlBar,
    ScanFields,
    SettingsFields,
    TopStatusStrip,
)

logger = logging.getLogger(__name__)


def _friendly_async_error(e: Exception) -> str:
    """把常见异常转成更友好的中文提示"""
    name = type(e).__name__
    if name == "NotConnectedError":
        return "尚未连接设备，请先在「连接」面板选择串口并点击连接。"
    if name == "SendTimeoutError":
        return "命令发送超时，设备未响应。请检查连接是否正常、固件是否运行。"
    if name == "NackError":
        return f"设备拒绝命令：{e}"
    return str(e)


# ---------------------------------------------------------------------------
# 跨线程信号桥：transport 回调线程 → UI 主线程
# ---------------------------------------------------------------------------

class _Bridge(QObject):
    status = Signal(object)
    scan_row = Signal(object)
    scan_end = Signal(object)
    approach_data = Signal(object)
    approach_done = Signal(object)
    error = Signal(object)
    # 日志：正文（类别由 _on_log_main 根据内容自动推断）
    log = Signal(str)
    # TX/RX 原始帧日志：(方向, 命令名, payload_hex)
    frame_log = Signal(str, str, str)
    connection = Signal(str)
    # 固件版本就绪：跨线程通知主线程更新 UI
    fw_version_ready = Signal(str)


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Cheaper STM")
        self.resize(1500, 950)

        # 设置窗口图标
        import os
        icon_path = os.path.join(os.path.dirname(__file__), "LOGO.ico")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))

        # 控制层
        self.config = TransportConfig()
        self.controller = STMController(self.config)
        self.scan_engine = ScanEngine()

        # 跨线程桥
        self.bridge = _Bridge()
        self._wire_bridge()

        # UI
        self._build_ui()
        self._wire_callbacks()
        self._start_timer()

    # -----------------------------------------------------------------
    # UI 构建
    # -----------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.central = central
        root = QVBoxLayout(central)
        root.setContentsMargins(2, 2, 2, 2)
        root.setSpacing(2)

        # 顶部状态条
        self.top_status = TopStatusStrip()
        root.addWidget(self.top_status)

        # 三栏主体
        splitter = QSplitter(Qt.Horizontal)

        # 左栏：可滚动参数区
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(2, 2, 2, 2)
        left_layout.setSpacing(2)

        # 各折叠组（只留 4 个）
        self.connection_panel = ConnectionPanel()
        self.feedback_fields = FeedbackFields()
        self.scan_fields = ScanFields()
        self.settings_fields = SettingsFields()

        sec_conn = CollapsibleSection("连接")
        sec_conn.add_widget(self.connection_panel)
        sec_scan = CollapsibleSection("扫描")
        sec_scan.add_widget(self.scan_fields)
        sec_fb = CollapsibleSection("反馈", expanded=False)
        sec_fb.add_widget(self.feedback_fields)
        sec_settings = CollapsibleSection("设置", expanded=False)
        sec_settings.add_widget(self.settings_fields)

        for sec in (sec_conn, sec_scan, sec_fb, sec_settings):
            left_layout.addWidget(sec)
        left_layout.addStretch()

        left_scroll.setWidget(left_container)
        left_scroll.setMinimumWidth(340)
        left_scroll.setMaximumWidth(420)

        # 中栏：图像 + 进度
        mid = QWidget()
        mid_layout = QVBoxLayout(mid)
        mid_layout.setContentsMargins(2, 2, 2, 2)
        self.image_panel = ImagePanel()
        self.progress = QProgressBar()
        self.progress.setFixedHeight(16)
        mid_layout.addWidget(self.image_panel, 1)
        mid_layout.addWidget(self.progress)
        self.manual_bar = ManualControlBar()
        self.manual_bar.bind(self.controller)
        # 连接扫描模式信号到手动控制抽屉里的模式标签
        self.scan_fields.mode_changed.connect(self.manual_bar.set_mode_label)
        mid_layout.addWidget(self.manual_bar)

        # 右栏：示波器 + 日志
        right = QSplitter(Qt.Vertical)
        self.osc_panel = LineOscilloscopePanel()
        self.log_panel = LogPanel()
        right.addWidget(self.osc_panel)
        right.addWidget(self.log_panel)
        right.setSizes([400, 250])

        splitter.addWidget(left_scroll)
        splitter.addWidget(mid)
        splitter.addWidget(right)
        splitter.setSizes([360, 780, 360])
        root.addWidget(splitter, 1)

        self.setCentralWidget(central)

        # 固定 ESTOP 按钮（已嵌入 TopStatusStrip）
        self.top_status.estop_btn.clicked.connect(self._on_estop)

        # 绑定 controller 给各参数面板
        self.feedback_fields.bind(self.controller)
        self.scan_fields.bind(self.controller)
        self.settings_fields.bind(self.controller)

    # -----------------------------------------------------------------
    # 信号桥与回调
    # -----------------------------------------------------------------

    def _wire_bridge(self) -> None:
        self.bridge.status.connect(self._on_status_main)
        self.bridge.scan_row.connect(self._on_scan_row_main)
        self.bridge.scan_end.connect(self._on_scan_end_main)
        self.bridge.approach_data.connect(self._on_approach_data_main)
        self.bridge.approach_done.connect(self._on_approach_done_main)
        self.bridge.error.connect(self._on_error_main)
        self.bridge.log.connect(self._on_log_main)
        self.bridge.frame_log.connect(self._on_frame_log_main)
        self.bridge.connection.connect(self._on_connection_main)
        self.bridge.fw_version_ready.connect(self._on_fw_version_main)

    def _wire_callbacks(self) -> None:
        self.controller.set_callbacks(
            status=lambda s: self.bridge.status.emit(s),
            scan_row=lambda r: self.bridge.scan_row.emit(r),
            scan_end=lambda e: self.bridge.scan_end.emit(e),
            approach_data=lambda d: self.bridge.approach_data.emit(d),
            approach_done=lambda d: self.bridge.approach_done.emit(d),
            error=lambda e: self.bridge.error.emit(e),
        )
        self.controller.transport.on_log = lambda msg: self.bridge.log.emit(msg)
        self.controller.transport.on_connection = lambda s: self.bridge.connection.emit(s)
        # TX/RX 原始帧日志
        self.controller.transport.on_frame_log = lambda d, n, p: self.bridge.frame_log.emit(d, n, p)

        # 连接面板
        self.connection_panel.connect_requested.connect(self._on_connect)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect)

        # 标定（通过 SettingsFields）
        self.settings_fields.apply_requested.connect(self._on_apply_calib)
        self.settings_fields.save_requested.connect(self._run_async(self.controller.save_calibration))
        self.settings_fields.load_requested.connect(self._run_async(self.controller.load_calibration))

        # 图像保存
        self.image_panel.save_requested.connect(self._on_save)

        # Approach 启停（启停按钮在 ScanFields 顶部，参数从 SettingsFields 取）
        self.scan_fields.approach_start_btn.clicked.connect(self._on_approach_start)
        self.scan_fields.approach_stop_btn.clicked.connect(self._run_async(self.controller.stop_approach))

    # -----------------------------------------------------------------
    # 定时刷新
    # -----------------------------------------------------------------

    def _start_timer(self) -> None:
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_tick)
        self.ui_timer.start(100)   # 10 FPS

    def _on_ui_tick(self) -> None:
        img = self.scan_engine.snapshot()
        if img is not None:
            self.image_panel.update_image(img)
            self.progress.setValue(int(self.scan_engine.progress() * 100))

    # -----------------------------------------------------------------
    # UI 主线程槽
    # -----------------------------------------------------------------

    def _on_status_main(self, s: Status) -> None:
        self.top_status.update_status(s)
        if s.is_locked and s.err != ErrCode.NONE:
            self._on_error_main(DeviceError(s.err, "设备进入锁定状态"))

    def _on_scan_row_main(self, row) -> None:
        self.scan_engine.on_row(row)
        # 实时更新示波器
        self.osc_panel.update_row(row.y_idx, row.I_data, row.z_data)

    def _on_scan_end_main(self, end) -> None:
        self.scan_engine.on_end(end.total_points)
        self.scan_fields.set_running(False)
        self.progress.setValue(100)
        self.bridge.log.emit(f"[扫描完成] 总点数 {end.total_points}")

    def _on_approach_data_main(self, d) -> None:
        # Approach 数据走日志（右栏示波器专用于扫描行）
        self.bridge.log.emit(
            f"[Approach] phase={'粗' if d.phase == 0 else '细'} "
            f"step/z={d.step_or_z} I={d.current_nA:.3f} nA"
        )

    def _on_approach_done_main(self, d) -> None:
        self.scan_fields.set_approach_running(False)
        self.bridge.log.emit(
            f"[Approach] {'成功' if d.success else '失败'} "
            f"步数={d.total_steps} 电流={d.final_current_nA:.3f} nA"
        )
        if d.success:
            QMessageBox.information(
                self, "Approach 成功",
                f"总步数 {d.total_steps}\n最终电流 {d.final_current_nA:.3f} nA\n\n"
                "已自动切回 idle，请手动开启反馈后扫描。"
            )
            try:
                self.controller.enter_idle()
            except Exception:
                pass
        else:
            QMessageBox.warning(
                self, "Approach 失败",
                f"总步数 {d.total_steps}\n最终电流 {d.final_current_nA:.3f} nA"
            )

    def _on_error_main(self, e: DeviceError) -> None:
        self.bridge.log.emit(f"[错误] {e}")
        if isinstance(e, FatalDeviceError):
            QMessageBox.critical(
                self, "致命错误",
                f"{e}\n\n下位机已自动 ESTOP。\n请检查硬件后通过「回 idle」解锁。"
            )

    def _on_log_main(self, msg: str) -> None:
        """根据消息内容自动推断日志类别"""
        category = "SYS"
        m = msg.lower()
        if msg.startswith("[错误]") or "失败" in msg or "异常" in msg or "nack" in m:
            category = "ERR"
        elif msg.startswith("[扫描") or "scan" in m or "行" in msg:
            category = "SCAN"
        elif msg.startswith("[Approach]") or "[连接]" in msg or "[固件]" in msg:
            category = "SYS"
        elif "警告" in msg or "warn" in m:
            category = "WARN"
        self.log_panel.append(msg, category)

    def _on_fw_version_main(self, version: str) -> None:
        """主线程槽：更新固件版本显示"""
        self.top_status.set_firmware_version(version)

    def _on_frame_log_main(self, direction: str, cmd_name: str, payload_hex: str) -> None:
        """TX/RX 原始帧日志

        Args:
            direction: "TX" / "RX"
            cmd_name: 命令名（如 STEP_MOTOR / ACK / NACK）
            payload_hex: payload 的 hex 字符串
        """
        if payload_hex:
            self.log_panel.append(f"{cmd_name}  payload={payload_hex}", direction)
        else:
            self.log_panel.append(f"{cmd_name}  (无 payload)", direction)

    def _on_connection_main(self, state: str) -> None:
        connected = state.startswith("connected")
        self.connection_panel.set_connected(connected)
        self.top_status.set_connected(connected)
        self.log_panel.append(f"[连接] {state}")

        if connected:
            def _get_version():
                try:
                    from protocol import Cmd, Reply, get_fw_version
                    version = self.controller.transport.query(
                        Cmd.GET_FW_VERSION, get_fw_version().payload, Reply.FW_VERSION,
                        timeout=3.0
                    )
                    if version:
                        self.bridge.fw_version_ready.emit(str(version))
                except Exception:
                    pass
            self._run_async(_get_version, "固件版本")()

        # 断连时复位操作
        if not connected:
            self.scan_fields.set_running(False)
            self.scan_fields.set_paused(False)
            self.scan_fields.set_approach_running(False)
            self.bridge.log.emit("[状态] 设备已断开，所有操作已停止")
            self._run_async(self.controller.transport.start_auto_reconnect, "自动重连")

    # -----------------------------------------------------------------
    # 按钮槽
    # -----------------------------------------------------------------

    def _on_estop(self) -> None:
        self.log_panel.append("[ESTOP] 用户触发急停")
        self.controller.estop()

    def _on_connect(self, port: str) -> None:
        if not port:
            QMessageBox.warning(self, "连接", "请先选择串口")
            return
        self.log_panel.append(f"[连接] 正在连接 {port} ...")
        self.connection_panel.connect_btn.setEnabled(False)
        self.connection_panel.disconnect_btn.setEnabled(False)
        def work():
            try:
                ok = self.controller.connect(port)
                if not ok:
                    self.bridge.log.emit(f"连接 {port} 失败: 无设备响应 (未收到 PONG)")
                    def alert():
                        QMessageBox.warning(self, "连接失败",
                            f"连接 {port} 失败，无设备响应。\n"
                            "请检查：\n1) 串口是否选对\n2) 固件是否烧录并正在运行\n"
                            "3) TX/RX 是否接反\n4) 设备是否被其他软件占用")
                        self.connection_panel.set_connected(False)
                    QTimer.singleShot(0, alert)
                else:
                    self.bridge.log.emit(f"连接 {port} 成功")
                    QTimer.singleShot(0, lambda: self.connection_panel.set_connected(True))
            except Exception as e:
                logger.exception("connect 异常")
                self.bridge.log.emit(f"连接异常: {e}")
                def alert():
                    QMessageBox.warning(self, "连接异常",
                        f"连接 {port} 时出错：\n{e}\n\n详细日志见 stm.log")
                    self.connection_panel.set_connected(False)
                QTimer.singleShot(0, alert)
        self._run_async(work, "连接")()

    def _on_disconnect(self) -> None:
        self.controller.disconnect()

    def _on_apply_calib(self, calib) -> None:
        def work():
            try:
                self.controller.set_calibration(
                    calib.tia_R, calib.x_nmV, calib.y_nmV, calib.z_nmV
                )
                self.bridge.log.emit(
                    f"标定已应用: R={calib.tia_R:.0f}Ω "
                    f"x={calib.x_nmV} y={calib.y_nmV} z={calib.z_nmV} nm/V"
                )
                # 同步示波器扫描宽度
                self.osc_panel.set_scan_params(self.scan_fields.nx.value())
            except Exception as e:
                self.bridge.log.emit(f"标定应用失败: {e}")
        self._run_async(work)()

    def _on_approach_start(self) -> None:
        params = self.settings_fields.build_approach_params()
        def work():
            try:
                self.controller.set_mode(Mode.APPROACH)
                self.controller.start_approach(params)
                self.bridge.log.emit("[Approach] 已启动")
            except Exception as e:
                self.bridge.log.emit(f"[Approach] 启动失败: {e}")
                self.bridge.approach_done.emit(type(
                    "D", (), {"success": False, "total_steps": 0,
                              "final_current_nA": 0.0})())
        self._run_async(work)()
        self.scan_fields.set_approach_running(True)

    def _on_save(self) -> None:
        img = self.scan_engine.snapshot()
        if img is None:
            QMessageBox.information(self, "保存", "暂无图像数据")
            return
        try:
            path = save_scan(img)
            QMessageBox.information(self, "保存成功", f"数据已保存到:\n{path}")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", str(e))

    # -----------------------------------------------------------------
    # 异步执行工具
    # -----------------------------------------------------------------

    def _run_async(self, fn, job_name: str = "操作"):
        if not hasattr(self, "_executor") or self._executor is None:
            import os
            self._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=min(4, (os.cpu_count() or 4)),
                thread_name_prefix="stm-worker"
            )
        def on_done(fut):
            try:
                fut.result()
            except Exception as e:
                logger.exception("%s 失败", job_name)
                msg = _friendly_async_error(e)
                self.bridge.log.emit(f"[{job_name}] 异常: {msg}")
                # 在主线程弹出提示（QTimer.singleShot(0) 会把调用排到主线程）
                QTimer.singleShot(0, lambda: QMessageBox.warning(self, job_name, msg))
        def runner():
            fut = self._executor.submit(fn)
            fut.add_done_callback(on_done)
        return runner

    # -----------------------------------------------------------------
    # 窗口大小变化
    # -----------------------------------------------------------------

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)

    # -----------------------------------------------------------------
    # 关闭
    # -----------------------------------------------------------------

    def closeEvent(self, event) -> None:
        try:
            if self.controller.is_connected:
                self.log_panel.append("断开连接中...")
                self.controller.disconnect()
        except Exception:
            pass
        super().closeEvent(event)
