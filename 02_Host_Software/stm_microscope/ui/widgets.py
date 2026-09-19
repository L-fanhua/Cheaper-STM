"""
STM 上位机 UI 组件库

布局理念：三栏专业式
  左栏：可折叠参数分组（同屏可见多组，不用切 Tab）
  中栏：主图像 + 进度
  右栏：扫描行示波器 + 日志
"""
from __future__ import annotations

import json
import logging
import pathlib
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, Signal, QTimer, QPoint, QRectF, QObject
from PySide6.QtGui import QFont, QMouseEvent, QColor, QPalette
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from controller import ApproachParams, Calibration, ScanParams, STMController
from protocol import ErrCode, Mode, ScanDirection, ScanMode, Status
from scanner import ScanImage, plane_level, line_level

logger = logging.getLogger(__name__)

pg.setConfigOption("background", "#1e1e1e")
pg.setConfigOption("foreground", "d")
pg.setConfigOption("imageAxisOrder", "row-major")


# ---------------------------------------------------------------------------
# 通用工具
# ---------------------------------------------------------------------------

class _NoWheelFilter(QObject):
    """拦截 SpinBox 的滚轮事件，防止误改"""
    def eventFilter(self, obj, event):
        if event.type() == event.Type.Wheel:
            return True  # 吞掉
        return False


_no_wheel_filter = _NoWheelFilter()


def dspin(value: float, mn: float, mx: float, step: float = 0.01,
          decimals: int = 4, suffix: str = "") -> QDoubleSpinBox:
    sb = QDoubleSpinBox()
    sb.setRange(mn, mx)
    sb.setSingleStep(step)
    sb.setDecimals(decimals)
    sb.setValue(value)
    if suffix:
        sb.setSuffix(suffix)
    sb.installEventFilter(_no_wheel_filter)
    return sb


def ispin(value: int, mn: int, mx: int, step: int = 1, suffix: str = "") -> QSpinBox:
    sb = QSpinBox()
    sb.setRange(mn, mx)
    sb.setSingleStep(step)
    sb.setValue(value)
    if suffix:
        sb.setSuffix(suffix)
    sb.installEventFilter(_no_wheel_filter)
    return sb


# ---------------------------------------------------------------------------
# 折叠分组：左侧参数区核心组件
# ---------------------------------------------------------------------------

class CollapsibleSection(QWidget):
    """
    可折叠的参数分组。
    标题栏点击展开/收起，内容区为 QFormLayout。
    """

    def __init__(self, title: str, parent: Optional[QWidget] = None, expanded: bool = True):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 2)
        layout.setSpacing(2)

        # 标题栏
        self.header = QToolButton()
        arrow = "▼" if expanded else "▶"
        self.header.setText(f"  {arrow}  {title}")
        self.header.setCheckable(True)
        self.header.setChecked(expanded)
        self.header.setStyleSheet(
            "QToolButton {"
            "  background: #2d2d30; color: #cccccc; border: none;"
            "  text-align: left; padding: 4px 6px; font-weight: bold;"
            "}"
            "QToolButton:hover { background: #3e3e42; }"
        )
        self.header.toggled.connect(self._on_toggle)

        # 内容容器
        self.content = QWidget()
        self.content.setVisible(expanded)
        self.form = QFormLayout(self.content)
        self.form.setContentsMargins(8, 4, 8, 4)
        self.form.setSpacing(4)

        layout.addWidget(self.header)
        layout.addWidget(self.content)

    def _on_toggle(self, checked: bool) -> None:
        arrow = "▼" if checked else "▶"
        # 保留原标题文本
        title = self.header.text().split("  ", 2)[-1]
        self.header.setText(f"  {arrow}  {title}")
        self.content.setVisible(checked)

    def add_row(self, label: str, widget: QWidget) -> None:
        self.form.addRow(label, widget)

    def add_widget(self, widget: QWidget) -> None:
        self.form.addRow(widget)


# ---------------------------------------------------------------------------
# 浮动 ESTOP 按钮（保留类定义兼容）
# ---------------------------------------------------------------------------

class FloatingEstopButton(QPushButton):
    """
    悬浮急停按钮（兼容保留，现用 TopStatusStrip 内嵌版）。
    """

    clicked_estop = Signal()

    def __init__(self, parent: QWidget):
        super().__init__("ESTOP", parent)
        self.setFixedSize(80, 80)
        self.setStyleSheet(
            "QPushButton {"
            "  background-color: rgba(220, 40, 40, 200);"
            "  color: white; font-weight: bold; font-size: 14px;"
            "  border-radius: 40px; border: 2px solid #ff8080;"
            "}"
            "QPushButton:hover { background-color: rgba(255, 60, 60, 230); }"
            "QPushButton:pressed { background-color: rgba(180, 20, 20, 255); }"
        )
        self.setToolTip("紧急停止 (拖动改变位置)")
        self._dragging = False
        self._drag_offset = QPoint()
        self._moved = False
        self.clicked.connect(self._on_click)

    def parent_resize(self) -> None:
        pass

    def mousePressEvent(self, e: QMouseEvent) -> None:
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_offset = e.position().toPoint()
            self._moved = False
            e.accept()
        else:
            super().mousePressEvent(e)

    def mouseMoveEvent(self, e: QMouseEvent) -> None:
        if self._dragging:
            e.accept()
        else:
            super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e: QMouseEvent) -> None:
        if self._dragging:
            self._dragging = False
            if self._moved:
                e.accept()
                return
        super().mouseReleaseEvent(e)

    def _on_click(self) -> None:
        if not self._moved:
            self.clicked_estop.emit()


# ---------------------------------------------------------------------------
# 各参数面板（嵌入折叠组用，无外框）
# ---------------------------------------------------------------------------

class ConnectionPanel(QWidget):
    """串口连接"""
    connect_requested = Signal(str)
    disconnect_requested = Signal()

    def __init__(self):
        super().__init__()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        self.disconnect_btn.setEnabled(False)
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: #ff6060;")

        row1 = QHBoxLayout()
        row1.addWidget(self.port_combo, 1)
        row1.addWidget(self.refresh_btn)
        row2 = QHBoxLayout()
        row2.addWidget(self.connect_btn)
        row2.addWidget(self.disconnect_btn)

        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.addLayout(row1)
        self._lay.addLayout(row2)
        self._lay.addWidget(self.status_label)
        self.refresh_ports()

    def refresh_ports(self) -> None:
        from transport import Transport
        self.port_combo.clear()
        ports = Transport.list_ports()
        for p in ports:
            self.port_combo.addItem(p)
        auto = Transport.auto_detect()
        if auto and auto not in ports:
            self.port_combo.insertItem(0, auto)
        if auto:
            self.port_combo.setCurrentText(auto)

    def _on_connect(self) -> None:
        port = self.port_combo.currentText().strip()
        if port:
            self.connect_requested.emit(port)

    def set_firmware_version(self, version: str) -> None:
        self.fw_version_lbl.setText(f"固件: {version}")

    def set_connected(self, connected: bool) -> None:
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        if connected:
            self.status_label.setText("已连接")
            self.status_label.setStyleSheet("color: #60ff60;")
        else:
            self.status_label.setText("未连接")
            self.status_label.setStyleSheet("color: #ff6060;")


class CalibrationFields(QWidget):
    """标定参数（嵌入折叠组）"""
    apply_requested = Signal(Calibration)
    save_requested = Signal()
    load_requested = Signal()

    def __init__(self):
        super().__init__()
        form = QFormLayout(self)
        form.setContentsMargins(0, 0, 0, 0)
        self.tia_R = dspin(100e6, 1e3, 1e10, step=10e6, decimals=0, suffix=" Ω")
        self.x_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.y_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.z_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.apply_btn = QPushButton("应用 (RAM)")
        self.save_btn = QPushButton("存 NVS")
        self.load_btn = QPushButton("加载")
        self.apply_btn.clicked.connect(self._on_apply)
        self.save_btn.clicked.connect(self.save_requested.emit)
        self.load_btn.clicked.connect(self.load_requested.emit)

        form.addRow("TIA R:", self.tia_R)
        form.addRow("x nm/V:", self.x_nmV)
        form.addRow("y nm/V:", self.y_nmV)
        form.addRow("z nm/V:", self.z_nmV)
        row = QHBoxLayout()
        row.addWidget(self.apply_btn)
        row.addWidget(self.save_btn)
        row.addWidget(self.load_btn)
        form.addRow(row)

    def _on_apply(self) -> None:
        self.apply_requested.emit(Calibration(
            tia_R=self.tia_R.value(),
            x_nmV=self.x_nmV.value(),
            y_nmV=self.y_nmV.value(),
            z_nmV=self.z_nmV.value(),
        ))


class FeedbackFields(QWidget):
    """反馈参数（精简：只保留扫描时必须实时调整的参数）"""
    def __init__(self):
        super().__init__()
        form = QFormLayout(self)
        form.setContentsMargins(0, 0, 0, 0)
        self.setpoint = dspin(1.0, 0.01, 100.0, step=0.1, decimals=3, suffix=" nA")
        self.kp = dspin(0.1, 0.0, 1000.0, decimals=4)
        self.ki = dspin(0.01, 0.0, 1000.0, decimals=4)
        self.kd = dspin(0.001, 0.0, 1000.0, decimals=5)
        self.apply_btn = QPushButton("应用参数")
        self.feedback_on_btn = QPushButton("开反馈")
        self.feedback_off_btn = QPushButton("回 idle")

        form.addRow("设定电流:", self.setpoint)
        form.addRow("Kp:", self.kp)
        form.addRow("Ki:", self.ki)
        form.addRow("Kd:", self.kd)
        form.addRow(self.apply_btn)
        row = QHBoxLayout()
        row.addWidget(self.feedback_on_btn)
        row.addWidget(self.feedback_off_btn)
        form.addRow(row)

    def bind(self, ctrl: STMController) -> None:
        def apply():
            try:
                ctrl.set_setpoint(self.setpoint.value())
                ctrl.set_pid(self.kp.value(), self.ki.value(), self.kd.value())
            except Exception as e:
                _warn(self, "应用失败", str(e))
        self.apply_btn.clicked.connect(apply)
        self.feedback_on_btn.clicked.connect(lambda: _safe(ctrl.enter_feedback, self))
        self.feedback_off_btn.clicked.connect(lambda: _safe(ctrl.enter_idle, self))


class ManualFields(QWidget):
    """手动定位"""
    def __init__(self):
        super().__init__()
        form = QFormLayout(self)
        form.setContentsMargins(0, 0, 0, 0)
        self.x = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.y = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.z = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.move_btn = QPushButton("移动 x/y/z")
        self.set_z_btn = QPushButton("设 z")
        self.retract_btn = QPushButton("全退 1000 步")

        form.addRow("x:", self.x)
        form.addRow("y:", self.y)
        form.addRow("z:", self.z)
        form.addRow(self.move_btn)
        form.addRow(self.set_z_btn)
        form.addRow(self.retract_btn)

    def bind(self, ctrl: STMController) -> None:
        self.move_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.move_xyz(self.x.value(), self.y.value(), self.z.value()), self))
        self.set_z_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.set_z(self.z.value()), self))
        self.retract_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.retract_z(1000), self))


class ApproachFields(QWidget):
    """Approach 参数"""
    def __init__(self):
        super().__init__()
        form = QFormLayout(self)
        form.setContentsMargins(0, 0, 0, 0)
        self.step_pulse = ispin(1, 1, 65535, suffix=" pulse")
        self.step_speed = ispin(100, 1, 100000, step=10, suffix=" sps")
        self.thresh_pre = dspin(0.1, 0.0, 100.0, step=0.01, decimals=3, suffix=" nA")
        self.thresh_lock = dspin(0.9, 0.0, 100.0, step=0.01, decimals=3, suffix=" nA")
        self.z_start = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.z_speed = dspin(1.0, 0.01, 100.0, step=0.1, decimals=3, suffix=" V/s")
        self.max_steps = ispin(10000, 1, 1000000, step=1000)
        self.max_steps.setToolTip(
            "最大累计步进脉冲数（安全上限）。\n"
            "≤ 65535: 旧 u16 协议格式，兼容所有固件；\n"
            "> 65535: 新 u32 格式，需固件同步更新，否则会被旧固件截断为低16位。"
        )
        self.retry = ispin(5, 0, 255)
        self.microstep = QComboBox()
        for ms in (1, 2, 4, 8, 16, 32, 64, 128, 256):
            self.microstep.addItem(f"1/{ms}", ms)
        self.microstep.setCurrentIndex(4)
        self.start_btn = QPushButton("开始 Approach")
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)

        form.addRow("单次脉冲:", self.step_pulse)
        form.addRow("步进速度:", self.step_speed)
        form.addRow("预检阈值:", self.thresh_pre)
        form.addRow("锁定阈值:", self.thresh_lock)
        form.addRow("z 起始:", self.z_start)
        form.addRow("z 速度:", self.z_speed)
        form.addRow("最大步数:", self.max_steps)
        form.addRow("重试脉冲:", self.retry)
        form.addRow("微步:", self.microstep)
        row = QHBoxLayout()
        row.addWidget(self.start_btn)
        row.addWidget(self.stop_btn)
        form.addRow(row)

    def build_params(self) -> ApproachParams:
        return ApproachParams(
            step_pulse=self.step_pulse.value(),
            step_speed_sps=self.step_speed.value(),
            thresh_pre_nA=self.thresh_pre.value(),
            thresh_lock_nA=self.thresh_lock.value(),
            z_start_V=self.z_start.value(),
            z_speed_Vps=self.z_speed.value(),
            max_steps=self.max_steps.value(),
            retry=self.retry.value(),
            microstep=self.microstep.currentData(),
        )

    def set_running(self, running: bool) -> None:
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)


class SettingsFields(QWidget):
    """设置面板（标定、Approach、反馈高级）"""
    apply_requested = Signal(Calibration)
    save_requested = Signal()
    load_requested = Signal()

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # 标定区
        calib_group = QGroupBox("标定")
        calib_form = QFormLayout(calib_group)
        calib_form.setContentsMargins(4, 4, 4, 4)
        calib_form.setSpacing(4)
        self.tia_R = dspin(100e6, 1e3, 1e10, step=10e6, decimals=0, suffix=" Ω")
        self.x_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.y_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.z_nmV = dspin(10.0, 0.0, 1000.0, step=1.0, decimals=2, suffix=" nm/V")
        self.calib_apply_btn = QPushButton("应用 (RAM)")
        self.calib_save_btn = QPushButton("存 NVS")
        self.calib_load_btn = QPushButton("加载")
        calib_form.addRow("TIA R:", self.tia_R)
        calib_form.addRow("x nm/V:", self.x_nmV)
        calib_form.addRow("y nm/V:", self.y_nmV)
        calib_form.addRow("z nm/V:", self.z_nmV)
        calib_row = QHBoxLayout()
        calib_row.addWidget(self.calib_apply_btn)
        calib_row.addWidget(self.calib_save_btn)
        calib_row.addWidget(self.calib_load_btn)
        calib_form.addRow(calib_row)

        # Approach区（仅参数输入框，启停按钮已移至 ScanFields）
        approach_group = QGroupBox("Approach")
        approach_form = QFormLayout(approach_group)
        approach_form.setContentsMargins(4, 4, 4, 4)
        approach_form.setSpacing(4)
        self.step_pulse = ispin(1, 1, 65535, suffix=" pulse")
        self.step_speed = ispin(100, 1, 100000, step=10, suffix=" sps")
        self.thresh_pre = dspin(0.1, 0.0, 100.0, step=0.01, decimals=3, suffix=" nA")
        self.thresh_lock = dspin(0.9, 0.0, 100.0, step=0.01, decimals=3, suffix=" nA")
        self.z_start = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.z_speed = dspin(1.0, 0.01, 100.0, step=0.1, decimals=3, suffix=" V/s")
        self.max_steps = ispin(10000, 1, 1000000, step=1000)
        self.max_steps.setToolTip(
            "最大累计步进脉冲数（安全上限）。\n"
            "≤ 65535: 旧 u16 协议格式，兼容所有固件；\n"
            "> 65535: 新 u32 格式，需固件同步更新，否则会被旧固件截断为低16位。"
        )
        self.retry = ispin(5, 0, 255)
        self.microstep = QComboBox()
        for ms in (1, 2, 4, 8, 16, 32, 64, 128, 256):
            self.microstep.addItem(f"1/{ms}", ms)
        self.microstep.setCurrentIndex(4)
        approach_form.addRow("单次脉冲:", self.step_pulse)
        approach_form.addRow("步进速度:", self.step_speed)
        approach_form.addRow("预检阈值:", self.thresh_pre)
        approach_form.addRow("锁定阈值:", self.thresh_lock)
        approach_form.addRow("z 起始:", self.z_start)
        approach_form.addRow("z 速度:", self.z_speed)
        approach_form.addRow("最大步数:", self.max_steps)
        approach_form.addRow("重试脉冲:", self.retry)
        approach_form.addRow("微步:", self.microstep)

        # 反馈高级区
        fb_adv_group = QGroupBox("反馈高级")
        fb_adv_form = QFormLayout(fb_adv_group)
        fb_adv_form.setContentsMargins(4, 4, 4, 4)
        fb_adv_form.setSpacing(4)
        self.pid_freq = ispin(1000, 100, 50000, step=100, suffix=" Hz")
        self.z_min = dspin(-15.0, -15.0, 15.0, step=0.5, decimals=2, suffix=" V")
        self.z_max = dspin(15.0, -15.0, 15.0, step=0.5, decimals=2, suffix=" V")
        self.overcurrent = dspin(10.0, 1.0, 200.0, step=1.0, decimals=1, suffix=" nA")
        self.fb_apply_btn = QPushButton("应用参数")
        fb_adv_form.addRow("PID 频率:", self.pid_freq)
        fb_adv_form.addRow("z 下限:", self.z_min)
        fb_adv_form.addRow("z 上限:", self.z_max)
        fb_adv_form.addRow("过流阈值:", self.overcurrent)
        fb_adv_form.addRow(self.fb_apply_btn)

        layout.addWidget(calib_group)
        layout.addWidget(approach_group)
        layout.addWidget(fb_adv_group)
        layout.addStretch()

    def _on_calib_apply(self) -> None:
        self.apply_requested.emit(Calibration(
            tia_R=self.tia_R.value(),
            x_nmV=self.x_nmV.value(),
            y_nmV=self.y_nmV.value(),
            z_nmV=self.z_nmV.value(),
        ))

    def build_params(self) -> ApproachParams:
        return ApproachParams(
            step_pulse=self.step_pulse.value(),
            step_speed_sps=self.step_speed.value(),
            thresh_pre_nA=self.thresh_pre.value(),
            thresh_lock_nA=self.thresh_lock.value(),
            z_start_V=self.z_start.value(),
            z_speed_Vps=self.z_speed.value(),
            max_steps=self.max_steps.value(),
            retry=self.retry.value(),
            microstep=self.microstep.currentData(),
        )

    def build_approach_params(self) -> ApproachParams:
        """包装内部 build_params，返回 ApproachParams"""
        return self.build_params()

    def bind(self, ctrl: STMController) -> None:
        # 标定
        self.calib_apply_btn.clicked.connect(self._on_calib_apply)
        self.calib_save_btn.clicked.connect(self.save_requested.emit)
        self.calib_load_btn.clicked.connect(self.load_requested.emit)

        # 反馈高级
        def apply_fb():
            try:
                ctrl.set_pid_freq(self.pid_freq.value())
                ctrl.set_z_limits(self.z_min.value(), self.z_max.value())
                ctrl.set_overcurrent(self.overcurrent.value())
            except Exception as e:
                _warn(self, "应用失败", str(e))
        self.fb_apply_btn.clicked.connect(apply_fb)


class ScanFields(QWidget):
    mode_changed = Signal(str, str)  # 模式名, 颜色

    """扫描参数（两页：常用 + 高级），带保存/加载配置文件"""
    start_requested = Signal(ScanParams)
    stop_requested = Signal()
    pause_requested = Signal()
    resume_requested = Signal()

    def __init__(self):
        super().__init__()
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(2)

        # ---------- 保存/加载条（放在顶部，永驻）----------
        preset_row = QHBoxLayout()
        self.save_btn = QPushButton("💾 保存配置")
        self.load_btn = QPushButton("📂 加载配置")
        self.preset_combo = QComboBox()
        self.preset_combo.setEditable(True)
        self.preset_combo.addItem("默认 粗扫 128")
        self.preset_combo.addItem("标准 256")
        self.preset_combo.addItem("高分辨 512")
        self.preset_combo.setToolTip("内置预设或最近加载的文件")
        preset_row.addWidget(QLabel("预设:"))
        preset_row.addWidget(self.preset_combo, 1)
        preset_row.addWidget(self.save_btn)
        preset_row.addWidget(self.load_btn)
        outer.addLayout(preset_row)

        # ---------- 快速 Approach 行（扫描前一键 approach）----------
        approach_row = QHBoxLayout()
        approach_row.addStretch()  # 按钮靠右
        self.approach_start_btn = QPushButton("▶ 开始 Approach")
        self.approach_start_btn.setMinimumHeight(30)
        self.approach_start_btn.setStyleSheet(
            "QPushButton { background: #5a4a2d; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background: #7a6a3d; }"
            "QPushButton:disabled { background: #555; }"
        )
        self.approach_stop_btn = QPushButton("■ 停止")
        self.approach_stop_btn.setEnabled(False)
        self.approach_stop_btn.setMinimumHeight(30)
        self.approach_stop_btn.setStyleSheet(
            "QPushButton { background: #8a2d2d; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background: #b03a3a; }"
        )
        approach_row.addWidget(self.approach_start_btn, 2)
        approach_row.addWidget(self.approach_stop_btn, 1)
        outer.addLayout(approach_row)

        # ---------- 两页 Tab ----------
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(False)
        outer.addWidget(self.tabs, 1)

        # ---- 常用页 ----
        common = QWidget()
        cf = QFormLayout(common)
        cf.setContentsMargins(4, 4, 4, 4)
        cf.setSpacing(4)
        self.cx = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.cy = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.range_x = dspin(1.0, 0.01, 30.0, step=0.1, decimals=3, suffix=" V")
        self.range_y = dspin(1.0, 0.01, 30.0, step=0.1, decimals=3, suffix=" V")
        self.nx = ispin(256, 1, 1024, step=16)
        self.ny = ispin(256, 1, 2048, step=16)
        self.speed = ispin(1000, 10, 10000, step=100, suffix=" pps")
        self.scan_mode = QComboBox()
        self.scan_mode.addItem("恒流 (PID 开)", ScanMode.CONSTANT_CURRENT)
        self.scan_mode.addItem("恒高 (PID 关, z 固定)", ScanMode.CONSTANT_HEIGHT)
        self.scan_mode.setCurrentIndex(0)
        self.z_height = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.z_height.setEnabled(False)  # 默认恒流，不需要
        self.ch_warning = QLabel("⚠ 恒高模式：z 固定不动，表面突起会撞 tip！仅用于原子级平坦表面。")
        self.ch_warning.setStyleSheet("color: #ff4040; font-size: 10px; font-weight: bold;")
        self.ch_warning.setVisible(False)
        self.direction = QComboBox()
        self.direction.addItem("正向", ScanDirection.FORWARD)
        self.direction.addItem("反向", ScanDirection.BACKWARD)
        self.direction.addItem("双向", ScanDirection.BIDIRECTIONAL)
        self.direction.setCurrentIndex(2)
        self.start_btn = QPushButton("▶ 开始扫描")
        self.start_btn.setMinimumHeight(34)
        self.start_btn.setStyleSheet(
            "QPushButton { background: #2d7c2d; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background: #3d9a3d; }"
            "QPushButton:disabled { background: #555; }"
        )
        self.stop_btn = QPushButton("■ 停止")
        self.stop_btn.setMinimumHeight(34)
        self.stop_btn.setStyleSheet(
            "QPushButton { background: #8a2d2d; color: white; font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background: #b03a3a; }"
        )
        self.stop_btn.setEnabled(False)
        self.pause_btn = QPushButton("⏸ 暂停")
        self.pause_btn.setEnabled(False)
        self.resume_btn = QPushButton("▶▶ 恢复")
        self.resume_btn.setEnabled(False)

        cf.addRow("中心 x:", self.cx)
        cf.addRow("中心 y:", self.cy)
        cf.addRow("x 范围:", self.range_x)
        cf.addRow("y 范围:", self.range_y)
        cf.addRow("x 像素:", self.nx)
        cf.addRow("y 像素:", self.ny)
        cf.addRow("采样速度:", self.speed)
        cf.addRow("扫描模式:", self.scan_mode)
        cf.addRow("z 固定:", self.z_height)
        cf.addRow(self.ch_warning)
        cf.addRow("方向:", self.direction)
        row1 = QHBoxLayout()
        row1.addWidget(self.start_btn, 2)
        row1.addWidget(self.stop_btn, 1)
        cf.addRow(row1)
        row2 = QHBoxLayout()
        row2.addWidget(self.pause_btn, 1)
        row2.addWidget(self.resume_btn, 1)
        cf.addRow(row2)
        self.tabs.addTab(common, "常用 ⚡")

        # ---- 高级页（不常用，防误触） ----
        advanced = QWidget()
        af = QFormLayout(advanced)
        af.setContentsMargins(4, 4, 4, 4)
        af.setSpacing(4)
        self.settle = ispin(10, 0, 1000, suffix=" ms")
        self.overshoot = dspin(0.1, 0.0, 5.0, step=0.05, decimals=3, suffix=" V")
        self.feedback_bw = ispin(1000, 100, 50000, step=100, suffix=" Hz")
        self.average = ispin(1, 1, 64, suffix="x")
        self.bidir_calib = QCheckBox("双向扫描后自动差分校准")
        self.bidir_calib.setChecked(False)
        self.preflight = QCheckBox("启动前预检电流")
        self.preflight.setChecked(True)

        warn = QLabel("⚠ 高级参数：修改不当会影响图像质量或 tip 寿命")
        warn.setStyleSheet("color: #ffaa00; font-size: 10px;")
        af.addRow(warn)
        af.addRow("行间 settling:", self.settle)
        af.addRow("预压电压:", self.overshoot)
        af.addRow("反馈带宽建议:", self.feedback_bw)
        af.addRow("采样平均:", self.average)
        af.addRow(self.bidir_calib)
        af.addRow(self.preflight)
        self.tabs.addTab(advanced, "高级 ⚙")

        # ---------- 信号 ----------
        self.save_btn.clicked.connect(self._on_save_preset)
        self.load_btn.clicked.connect(self._on_load_preset)
        self.preset_combo.currentIndexChanged.connect(self._on_preset_selected)
        self.scan_mode.currentIndexChanged.connect(self._on_scan_mode_changed)

    # ---- 扫描模式切换 ----
    def _on_scan_mode_changed(self, idx: int) -> None:
        is_ch = self.scan_mode.currentData() == ScanMode.CONSTANT_HEIGHT
        self.z_height.setEnabled(is_ch)
        self.ch_warning.setVisible(is_ch)
        if hasattr(self, 'mode_changed'):
            if is_ch:
                self.mode_changed.emit('恒高', '#ff4040')
            else:
                self.mode_changed.emit('恒流', '#60ff60')

    # ---- 配置/预设 ----
    def _config_dict(self) -> dict:
        return {
            "version": 1,
            "scan": {
                "cx_V":   self.cx.value(),
                "cy_V":   self.cy.value(),
                "range_x_V": self.range_x.value(),
                "range_y_V": self.range_y.value(),
                "nx":     self.nx.value(),
                "ny":     self.ny.value(),
                "speed_pps": self.speed.value(),
                "direction_idx": self.direction.currentIndex(),
                "scan_mode_idx": self.scan_mode.currentIndex(),
                "z_height_V": self.z_height.value(),
            },
            "advanced": {
                "settle_ms": self.settle.value(),
                "overshoot_V": self.overshoot.value(),
                "feedback_bw_hz": self.feedback_bw.value(),
                "average": self.average.value(),
                "bidir_calib": self.bidir_calib.isChecked(),
                "preflight": self.preflight.isChecked(),
            },
        }

    def _load_config_dict(self, d: dict) -> None:
        s = d.get("scan", {})
        a = d.get("advanced", {})
        # 常用
        for k, w in [("cx_V", self.cx), ("cy_V", self.cy),
                      ("range_x_V", self.range_x), ("range_y_V", self.range_y)]:
            if k in s: w.setValue(float(s[k]))
        for k, w in [("nx", self.nx), ("ny", self.ny), ("speed_pps", self.speed)]:
            if k in s: w.setValue(int(s[k]))
        if "direction_idx" in s:
            self.direction.setCurrentIndex(int(s["direction_idx"]))
        if "scan_mode_idx" in s:
            self.scan_mode.setCurrentIndex(int(s["scan_mode_idx"]))
        if "z_height_V" in s:
            self.z_height.setValue(float(s["z_height_V"]))
        # 高级
        if "settle_ms" in a:     self.settle.setValue(int(a["settle_ms"]))
        if "overshoot_V" in a:   self.overshoot.setValue(float(a["overshoot_V"]))
        if "feedback_bw_hz" in a: self.feedback_bw.setValue(int(a["feedback_bw_hz"]))
        if "average" in a:       self.average.setValue(int(a["average"]))
        if "bidir_calib" in a:   self.bidir_calib.setChecked(bool(a["bidir_calib"]))
        if "preflight" in a:     self.preflight.setChecked(bool(a["preflight"]))

    _BUILTIN_PRESETS = {
        "默认 粗扫 128": dict(version=1,
            scan=dict(cx_V=0,cy_V=0,range_x_V=1.0,range_y_V=1.0,nx=128,ny=128,speed_pps=2000,direction_idx=2),
            advanced=dict(settle_ms=20,overshoot_V=0.1,feedback_bw_hz=2000,average=1,bidir_calib=False,preflight=True)),
        "标准 256": dict(version=1,
            scan=dict(cx_V=0,cy_V=0,range_x_V=1.0,range_y_V=1.0,nx=256,ny=256,speed_pps=1000,direction_idx=2),
            advanced=dict(settle_ms=10,overshoot_V=0.1,feedback_bw_hz=1000,average=1,bidir_calib=False,preflight=True)),
        "高分辨 512": dict(version=1,
            scan=dict(cx_V=0,cy_V=0,range_x_V=0.5,range_y_V=0.5,nx=512,ny=512,speed_pps=500,direction_idx=2),
            advanced=dict(settle_ms=30,overshoot_V=0.2,feedback_bw_hz=500,average=2,bidir_calib=True,preflight=True)),
    }

    def _on_preset_selected(self, idx: int) -> None:
        key = self.preset_combo.currentText()
        if key in self._BUILTIN_PRESETS:
            self._load_config_dict(self._BUILTIN_PRESETS[key])

    def _on_save_preset(self) -> None:
        import json, time
        default = f"scan_preset_{time.strftime('%Y%m%d_%H%M%S')}.json"
        path, _ = QFileDialog.getSaveFileName(self, "保存扫描配置", default, "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._config_dict(), f, indent=2, ensure_ascii=False)
            # 加到下拉
            name = pathlib.PurePath(path).stem
            if name not in [self.preset_combo.itemText(i) for i in range(self.preset_combo.count())]:
                self.preset_combo.insertItem(0, name)
                self._recent_path = path
            QMessageBox.information(self, "保存成功", f"配置已写入:\n{path}")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", str(e))

    def _on_load_preset(self) -> None:
        import json
        path, _ = QFileDialog.getOpenFileName(self, "加载扫描配置", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                d = json.load(f)
            self._load_config_dict(d)
            name = pathlib.PurePath(path).stem
            if name not in [self.preset_combo.itemText(i) for i in range(self.preset_combo.count())]:
                self.preset_combo.insertItem(0, name)
            self.preset_combo.setCurrentText(name)
            QMessageBox.information(self, "加载成功", f"已应用配置:\n{path}")
        except Exception as e:
            QMessageBox.warning(self, "加载失败", str(e))

    # ---- 与旧接口兼容 ----
    def bind(self, ctrl: STMController) -> None:
        def start():
            # 设置扫描模式
            try:
                ctrl.set_scan_mode(self.scan_mode.currentData())
            except Exception as e:
                _warn(self, "设置扫描模式失败", str(e))
                return
            if self.scan_mode.currentData() == ScanMode.CONSTANT_HEIGHT:
                try:
                    ctrl.set_z(self.z_height.value())
                except Exception as e:
                    _warn(self, "设置 z 高度失败", str(e))
                    return
            params = ScanParams(
                cx_V=self.cx.value(), cy_V=self.cy.value(),
                range_x_V=self.range_x.value(), range_y_V=self.range_y.value(),
                nx=self.nx.value(), ny=self.ny.value(),
                speed_pps=self.speed.value(),
                direction=self.direction.currentData(),
                settle_ms=self.settle.value(),
                overshoot_V=self.overshoot.value(),
            )
            _safe(lambda: ctrl.start_scan(params), self)
        self.start_btn.clicked.connect(start)
        self.stop_btn.clicked.connect(lambda: _safe(ctrl.stop_scan, self))
        self.pause_btn.clicked.connect(lambda: _safe(ctrl.pause_scan, self))
        self.resume_btn.clicked.connect(lambda: _safe(ctrl.resume_scan, self))

    def set_running(self, running: bool) -> None:
        self.start_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        self.pause_btn.setEnabled(running)
        self.resume_btn.setEnabled(False)

    def set_paused(self, paused: bool) -> None:
        self.pause_btn.setEnabled(not paused)
        self.resume_btn.setEnabled(paused)

    def set_approach_running(self, running: bool) -> None:
        """切换 Approach 启停按钮的可用状态"""
        self.approach_start_btn.setEnabled(not running)
        self.approach_stop_btn.setEnabled(running)


# ---------------------------------------------------------------------------
# 中栏底部：手动控制抽屉
# ---------------------------------------------------------------------------


class ManualControlBar(QWidget):
    """
    中栏底部手动控制抽屉。
    可收起的横向控制条：标题栏点击展开/收起，展开后显示 4 个区。
    默认收起。
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # 标题栏
        self.header = QToolButton()
        self.header.setText("  ▶  手动控制")
        self.header.setCheckable(True)
        self.header.setChecked(False)
        self.header.setStyleSheet(
            "QToolButton {"
            "  background: #2d2d30; color: #cccccc; border: none;"
            "  text-align: left; padding: 3px 8px; font-weight: bold;"
            "}"
            "QToolButton:hover { background: #3e3e42; }"
        )
        self.header.toggled.connect(self._on_toggle)
        root.addWidget(self.header)

        # 内容容器
        self.content = QWidget()
        self.content.setVisible(False)
        content_lay = QHBoxLayout(self.content)
        content_lay.setContentsMargins(4, 4, 4, 4)
        content_lay.setSpacing(6)

        # 区1 - 压电定位
        z1 = self._make_zone("压电定位")
        self.x_spin = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.y_spin = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.z_spin = dspin(0.0, -15.0, 15.0, step=0.1, decimals=3, suffix=" V")
        self.move_xyz_btn = QPushButton("移动 xyz")
        self.set_z_btn = QPushButton("设 z")
        f1 = QFormLayout()
        f1.setContentsMargins(0, 0, 0, 0)
        f1.setSpacing(2)
        f1.addRow("x:", self.x_spin)
        f1.addRow("y:", self.y_spin)
        f1.addRow("z:", self.z_spin)
        r1 = QHBoxLayout()
        r1.addWidget(self.move_xyz_btn)
        r1.addWidget(self.set_z_btn)
        f1.addRow(r1)
        z1.layout().addLayout(f1)

        # 区2 - 步进电机
        z2 = self._make_zone("步进电机")
        self.step_spin = ispin(10, 1, 20000)
        self.speed_spin = ispin(2000, 1000, 10000, step=100, suffix=" sps")
        self.fwd_btn = QPushButton("进")
        self.bwd_btn = QPushButton("退")
        self.full_retract_btn = QPushButton("全退")
        f2 = QFormLayout()
        f2.setContentsMargins(0, 0, 0, 0)
        f2.setSpacing(2)
        f2.addRow("步数:", self.step_spin)
        f2.addRow("速度:", self.speed_spin)
        r2 = QHBoxLayout()
        r2.addWidget(self.fwd_btn)
        r2.addWidget(self.bwd_btn)
        r2.addWidget(self.full_retract_btn)
        f2.addRow(r2)
        z2.layout().addLayout(f2)

        # 区3 - 样品偏压
        z3 = self._make_zone("样品偏压")
        self.bias_spin = dspin(0.0, -10.0, 10.0, step=0.1, decimals=3, suffix=" V")
        self.bias_apply_btn = QPushButton("应用")
        self.bias_toggle_btn = QPushButton("偏压: 关")
        self.bias_toggle_btn.setCheckable(True)
        f3 = QFormLayout()
        f3.setContentsMargins(0, 0, 0, 0)
        f3.setSpacing(2)
        f3.addRow("偏压:", self.bias_spin)
        r3 = QHBoxLayout()
        r3.addWidget(self.bias_apply_btn)
        r3.addWidget(self.bias_toggle_btn)
        f3.addRow(r3)
        z3.layout().addLayout(f3)

        # 区4 - 反馈
        z4 = self._make_zone("反馈")
        self.fb_on_btn = QPushButton("开反馈")
        self.fb_off_btn = QPushButton("关反馈")
        self.mode_lbl = QLabel("恒流")
        self.mode_lbl.setStyleSheet("color: #60ff60; font-size: 10px;")
        f4 = QVBoxLayout()
        f4.setContentsMargins(0, 0, 0, 0)
        f4.setSpacing(2)
        f4.addWidget(self.fb_on_btn)
        f4.addWidget(self.fb_off_btn)
        f4.addWidget(self.mode_lbl)
        f4.addStretch()
        z4.layout().addLayout(f4)

        for z in (z1, z2, z3, z4):
            content_lay.addWidget(z)
        content_lay.addStretch()
        root.addWidget(self.content)

        self.zones = [z1, z2, z3, z4]

    def _make_zone(self, title: str) -> QFrame:
        """创建一个带标题的浅色边框分区"""
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { border: 1px solid #3c3c3c; border-radius: 3px; }"
        )
        lay = QVBoxLayout(frame)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(2)
        lbl = QLabel(title)
        lbl.setStyleSheet("color: #9a9a9a; font-size: 10px; font-weight: bold;")
        lay.addWidget(lbl)
        return frame

    def _on_toggle(self, checked: bool) -> None:
        arrow = "▼" if checked else "▶"
        self.header.setText(f"  {arrow}  手动控制")
        self.content.setVisible(checked)

    def bind(self, ctrl: STMController) -> None:
        """把所有按钮连接到 controller"""
        self.move_xyz_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.move_xyz(
                self.x_spin.value(), self.y_spin.value(), self.z_spin.value()), self))
        self.set_z_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.set_z(self.z_spin.value()), self))
        def on_step(direction: int):
            """direction: +1=进, -1=退

            固件 STEP_MOTOR 的 steps 字段是无符号 u16，
            不能用负数表示方向（会变成 65526 触发 PARAM_OUT_OF_RANGE）。
            「进」走 STEP_MOTOR（正数步数），「退」走 RETRACT_Z（专门的后退命令）。
            """
            n = self.step_spin.value()
            try:
                if direction > 0:
                    # 进：STEP_MOTOR 发送正数步数
                    ctrl.step_motor(n, self.speed_spin.value())
                else:
                    # 退：用 RETRACT_Z（固件专门的后退命令，带速度）
                    ctrl.retract_z(n, self.speed_spin.value())
            except Exception as e:
                msg = str(e)
                if "PARAM_OUT_OF_RANGE" in msg or "参数越界" in msg:
                    _warn(self, "步进电机参数被拒绝",
                          f"设备拒绝命令（参数越界）。\n\n"
                          f"当前：步数={n}，速度={self.speed_spin.value()} sps\n\n"
                          f"payload 诊断见 stm.log。")
                else:
                    _warn(self, "步进电机操作失败", msg)

        self.fwd_btn.clicked.connect(lambda: on_step(+1))
        self.bwd_btn.clicked.connect(lambda: on_step(-1))
        self.full_retract_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.retract_z(1000), self))
        self.bias_apply_btn.clicked.connect(
            lambda: _safe(lambda: ctrl.set_bias(self.bias_spin.value()), self))

        def on_bias_toggle(checked: bool) -> None:
            if checked:
                self.bias_toggle_btn.setText("偏压: 开")
                _safe(lambda: ctrl.set_bias(self.bias_spin.value()), self)
            else:
                self.bias_toggle_btn.setText("偏压: 关")
                _safe(lambda: ctrl.set_bias(0.0), self)
        self.bias_toggle_btn.toggled.connect(on_bias_toggle)

        def on_fb_on():
            try:
                ctrl.enter_feedback()
                self.set_mode_label('恒流', '#60ff60')
            except Exception as e:
                _warn(self, "操作失败", str(e))

        def on_fb_off():
            try:
                ctrl.enter_idle()
                self.set_mode_label('idle', '#909090')
            except Exception as e:
                _warn(self, "操作失败", str(e))

        self.fb_on_btn.clicked.connect(on_fb_on)
        self.fb_off_btn.clicked.connect(on_fb_off)


# ---------------------------------------------------------------------------
# 中栏：图像 + 进度
# ---------------------------------------------------------------------------

    def set_mode_label(self, name: str, color: str) -> None:
        """外部调用，更新反馈区下方的扫描模式标签"""
        self.mode_lbl.setText(name)
        self.mode_lbl.setStyleSheet(f'color: {color}; font-size: 10px;')

class ImagePanel(QWidget):
    """
    主图像面板。
    显示 z 地形图（恒流模式主信号）。
    实时处理预览：原始 / 行级去背景 / 平面去背景。
    """
    save_requested = Signal()

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 顶部控制条
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("处理:"))
        self.process = QComboBox()
        self.process.addItem("原始")
        self.process.addItem("行级去背景")
        self.process.addItem("平面去背景")
        self.process.currentIndexChanged.connect(self._refresh)
        ctrl.addWidget(self.process)

        ctrl.addWidget(QLabel("色标:"))
        self.cmap = QComboBox()
        for name in ("viridis", "plasma", "inferno", "magma", "gray"):
            self.cmap.addItem(name)
        self.cmap.currentIndexChanged.connect(self._set_colormap)
        ctrl.addWidget(self.cmap)

        self.auto_levels = QCheckBox("自动色阶")
        self.auto_levels.setChecked(True)
        self.auto_levels.toggled.connect(self._refresh)
        ctrl.addWidget(self.auto_levels)

        self.save_btn = QPushButton("保存")
        self.save_btn.clicked.connect(self.save_requested.emit)
        ctrl.addWidget(self.save_btn)
        ctrl.addStretch()

        self.info_label = QLabel("-")
        ctrl.addWidget(self.info_label)
        layout.addLayout(ctrl)

        # 图像 + 色标
        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(False)
        self.plot.setLabel("left", "y (像素)")
        self.plot.setLabel("bottom", "x (像素)")
        self.plot.showAxis("top", True)
        self.plot.showAxis("right", True)
        self.img_item = pg.ImageItem()
        self.plot.addItem(self.img_item)

        self.hist = pg.HistogramLUTItem()
        self.hist.setImageItem(self.img_item)
        self.hist.gradient.loadPreset("viridis")

        # HistogramLUTItem 是 graphics item，需放入 GraphicsLayoutWidget 才能加入布局
        self.hist_widget = pg.GraphicsLayoutWidget()
        self.hist_widget.addItem(self.hist)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.plot)
        splitter.addWidget(self.hist_widget)
        splitter.setSizes([900, 120])
        layout.addWidget(splitter, 1)

        self._image: Optional[ScanImage] = None
        self._set_colormap()

    def update_image(self, image: ScanImage) -> None:
        self._image = image
        self._refresh()

    def _refresh(self) -> None:
        if self._image is None:
            return
        data = self._image.z_image
        proc = self.process.currentIndex()
        if proc == 1:
            data = line_level(data)
        elif proc == 2:
            data = plane_level(data)

        finite = np.isfinite(data)
        if finite.any():
            vmin = float(np.nanmin(data[finite])) if not self.auto_levels.isChecked() else None
            vmax = float(np.nanmax(data[finite])) if not self.auto_levels.isChecked() else None
            data = np.where(finite, data, np.nanmin(data[finite]))
        else:
            data = np.zeros_like(data)
            vmin = vmax = None

        self.img_item.setImage(data, autoLevels=self.auto_levels.isChecked())
        if not self.auto_levels.isChecked() and vmin is not None:
            self.img_item.setLevels([vmin, vmax])

        # 信息标签
        calib = self._image.calib
        x_nm = self._image.x_range_nm()
        y_nm = self._image.y_range_nm()
        rows = self._image.rows_received
        ny = self._image.ny
        self.info_label.setText(
            f"{rows}/{ny} 行  |  范围: {x_nm:.1f} × {y_nm:.1f} nm  "
            f"({self._image.nx}×{self._image.ny})"
        )

    def _set_colormap(self) -> None:
        try:
            self.hist.gradient.loadPreset(self.cmap.currentText())
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 右栏：扫描行示波器 + 日志
# ---------------------------------------------------------------------------

class LineOscilloscopePanel(QWidget):
    """
    扫描行示波器。
    显示当前扫描行的 I(x) 和 z(x) 双曲线，双 y 轴。
    用于实时判断反馈稳定性、噪声水平、压电迟滞。
    """
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 双 y 轴 PlotWidget
        self.plot = pg.PlotWidget(title="当前扫描行")
        self.plot.setLabel("bottom", "x 像素")
        self.plot.setLabel("left", "z (V)", color="#60ff60")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.z_curve = self.plot.plot(pen=pg.mkPen("#60ff60", width=2), name="z")
        layout.addWidget(self.plot, 1)

        # 右侧 y 轴（电流）
        self.plot.showAxis("right")
        self.plot.getAxis("right").setLabel("I (nA)", color="#60a0ff")
        # 用 ViewBox 实现双 y 轴
        self.I_view = pg.ViewBox()
        self.plot.showAxis("right")
        self.plot.scene().addItem(self.I_view)
        self.plot.getAxis("right").linkToView(self.I_view)
        self.I_view.setXLink(self.plot)
        self.I_curve = pg.PlotCurveItem(pen=pg.mkPen("#60a0ff", width=2), name="I")
        self.I_view.addItem(self.I_curve)

        # 更新 ViewBox 位置
        def update_views():
            self.I_view.setGeometry(self.plot.getViewBox().sceneBoundingRect())
            self.I_view.linkedViewChanged(self.plot.getViewBox(), self.I_view.XAxis)
        self.plot.getViewBox().sigResized.connect(update_views)

        # 行号标签
        self.row_label = QLabel("等待扫描...")
        self.row_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.row_label)

        self._nx = 0

    def set_scan_params(self, nx: int) -> None:
        self._nx = nx

    def update_row(self, y_idx: int, I_data, z_data) -> None:
        """更新当前行曲线"""
        x = np.arange(len(I_data))
        self.z_curve.setData(x, z_data)
        self.I_curve.setData(x, I_data)
        self.row_label.setText(f"第 {y_idx} 行")

    def clear(self) -> None:
        self.z_curve.setData([], [])
        self.I_curve.setData([], [])
        self.row_label.setText("等待扫描...")


# 日志类别 → 颜色 + 标签
LOG_STYLES = {
    "TX":   "#60a0ff",   # 发送（蓝）
    "RX":   "#60ff60",   # 接收/ACK（绿）
    "SYS":  "#cccccc",   # 系统（灰白）
    "WARN": "#ffaa00",   # 警告（橙）
    "ERR":  "#ff4040",   # 错误（红）
    "SCAN": "#a0a0ff",   # 扫描数据（淡蓝紫）
}


class LogPanel(QWidget):
    """统一格式的日志输出面板

    日志行格式：
        HH:MM:SS.mmm [TX  ] CMD_NAME payload_hex
        HH:MM:SS.mmm [RX  ] ACK/NACK/STATUS ...
        HH:MM:SS.mmm [SYS ] 连接 COM4 成功
        HH:MM:SS.mmm [ERR ] 设备拒绝：参数越界
    """
    MAX_LINES = 2000   # 超过自动丢弃旧行，防内存膨胀

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # 工具条：清空 / 自动滚动
        bar = QHBoxLayout()
        bar.setContentsMargins(4, 2, 4, 2)
        bar.addWidget(QLabel("日志"))
        bar.addStretch()
        self.autoscroll_chk = QCheckBox("自动滚动")
        self.autoscroll_chk.setChecked(True)
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self.clear)
        bar.addWidget(self.autoscroll_chk)
        bar.addWidget(self.clear_btn)
        layout.addLayout(bar)

        self.text = QTextEdit()
        self.text.setReadOnly(True)
        self.text.setFont(QFont("Consolas", 9))
        # 深色背景
        self.text.setStyleSheet(
            "QTextEdit { background: #1e1e1e; color: #cccccc; }"
        )
        layout.addWidget(self.text)

        self._line_count = 0

    def _ts(self) -> str:
        """当前时间戳 HH:MM:SS.mmm"""
        t = __import__("datetime").datetime.now()
        return t.strftime("%H:%M:%S.") + f"{t.microsecond // 1000:03d}"

    def append(self, msg: str, category: str = "SYS") -> None:
        """追加一条日志

        Args:
            msg: 日志正文
            category: TX/RX/SYS/WARN/ERR/SCAN
        """
        color = LOG_STYLES.get(category, "#cccccc")
        ts = self._ts()
        tag = f"[{category:<4}]"
        # 用 HTML 着色，标签按类别上色，正文统一浅灰
        html = (
            f'<span style="color:#808080;">{ts}</span> '
            f'<span style="color:{color};font-weight:bold;">{tag}</span> '
            f'<span style="color:#dcdcdc;">{msg}</span>'
        )
        self.text.append(html)
        self._line_count += 1
        # 超限时移除旧行（QTextEdit 没有 trim API，用 cursor 删除）
        if self._line_count > self.MAX_LINES:
            from PySide6.QtGui import QTextCursor
            cursor = self.text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor,
                                self._line_count - self.MAX_LINES)
            cursor.removeSelectedText()
            cursor.deleteChar()
            self._line_count = self.MAX_LINES
        if self.autoscroll_chk.isChecked():
            sb = self.text.verticalScrollBar()
            sb.setValue(sb.maximum())

    # ---- 便捷方法 ----
    def log_tx(self, msg: str) -> None:
        self.append(msg, "TX")

    def log_rx(self, msg: str) -> None:
        self.append(msg, "RX")

    def log_sys(self, msg: str) -> None:
        self.append(msg, "SYS")

    def log_warn(self, msg: str) -> None:
        self.append(msg, "WARN")

    def log_err(self, msg: str) -> None:
        self.append(msg, "ERR")

    def log_scan(self, msg: str) -> None:
        self.append(msg, "SCAN")

    def clear(self) -> None:
        self.text.clear()
        self._line_count = 0


# ---------------------------------------------------------------------------
# 状态条（顶部细条，永驻）
# ---------------------------------------------------------------------------

class TopStatusStrip(QWidget):
    """顶部状态条：模式/x/y/z/样品电流/步数 + 内嵌 ESTOP 大红钮"""
    def __init__(self):
        super().__init__()
        self.setFixedHeight(40)
        self.setStyleSheet("background: #252526;")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(8)

        # 使用等宽字体，固定宽度，防止数据变化导致标签位移
        mono_font = QFont("Consolas", 10)
        mono_font.setStyleHint(QFont.Monospace)

        def _fixed_label(initial: str, width: int) -> QLabel:
            lbl = QLabel(initial)
            lbl.setFont(mono_font)
            lbl.setFixedWidth(width)
            lbl.setStyleSheet("color: #e0e0e0; font-size: 12px;")
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            return lbl

        # 左侧状态标签：模式 / x / y / z / 样品(nA) / 步数（固定宽度）
        self.mode_lbl = _fixed_label("模式: ----", 100)
        self.x_lbl = _fixed_label("x: ----V", 100)
        self.y_lbl = _fixed_label("y: ----V", 100)
        self.z_lbl = _fixed_label("z: ----V", 100)
        self.sample_lbl = _fixed_label("样品: ----nA", 120)
        self.step_lbl = _fixed_label("步: ----", 90)
        self.fw_version_lbl = _fixed_label("固件: --", 110)
        self.fw_version_lbl.setStyleSheet("color: #909090; font-size: 11px;")
        for w in (self.mode_lbl, self.x_lbl, self.y_lbl, self.z_lbl,
                  self.sample_lbl, self.step_lbl, self.fw_version_lbl):
            layout.addWidget(w)

        # 中间 stretch
        layout.addStretch()

        # 右侧：ESTOP 按钮、间隔8、err_lbl、conn_lbl
        self.estop_btn = QPushButton("ESTOP")
        self.estop_btn.setFixedSize(88, 30)
        self.estop_btn.setStyleSheet(
            "QPushButton {"
            "  background: #c02020; color: white; font-weight: bold;"
            "  border: 2px solid #ff4040; border-radius: 4px;"
            "}"
            "QPushButton:hover { background: #ff3030; }"
            "QPushButton:pressed { background: #801010; }"
        )
        layout.addWidget(self.estop_btn)
        layout.addSpacing(8)

        self.err_lbl = QLabel("●")
        self.err_lbl.setStyleSheet("color: #60ff60; font-size: 18px;")
        self.conn_lbl = QLabel("○")
        self.conn_lbl.setStyleSheet("color: #606060; font-size: 18px;")

        layout.addWidget(self.err_lbl)
        layout.addWidget(self.conn_lbl)

    def update_status(self, s: Status) -> None:
        from protocol import ERR_DESCRIPTIONS  # 延迟导入避免循环
        self.mode_lbl.setText(f"模式: {s.mode.name}")
        self.x_lbl.setText(f"x: {s.x_V:+.3f} V")
        self.y_lbl.setText(f"y: {s.y_V:+.3f} V")
        self.z_lbl.setText(f"z: {s.z_V:+.3f} V")
        # 样品信号 = TIA 输出电流（nA）；TIA 输出电压与电流成正比，nA 数值即相对电压
        self.sample_lbl.setText(f"样品: {s.current_nA:.3f} nA")
        self.step_lbl.setText(f"步: {s.step_pos}")
        if s.err == ErrCode.NONE:
            self.err_lbl.setText("●")
            self.err_lbl.setStyleSheet("color: #60ff60; font-size: 18px;")
            self.err_lbl.setToolTip("无错误")
        else:
            desc = ERR_DESCRIPTIONS.get(s.err, "未知")
            self.err_lbl.setText("●")
            self.err_lbl.setStyleSheet("color: #ff4040; font-size: 18px;")
            self.err_lbl.setToolTip(f"错误: {desc}")

    def set_firmware_version(self, version: str) -> None:
        self.fw_version_lbl.setText(f"固件: {version}")

    def set_connected(self, connected: bool) -> None:
        if connected:
            self.conn_lbl.setText("●")
            self.conn_lbl.setStyleSheet("color: #60ff60; font-size: 18px;")
            self.conn_lbl.setToolTip("已连接")
        else:
            self.conn_lbl.setText("○")
            self.conn_lbl.setStyleSheet("color: #606060; font-size: 18px;")
            self.conn_lbl.setToolTip("未连接")


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------

def _safe(fn, parent: QWidget) -> None:
    try:
        fn()
    except Exception as e:
        _warn(parent, "操作失败", str(e))


def _friendly_error(e: Exception) -> str:
    """把常见异常转成更友好的中文提示"""
    name = type(e).__name__
    if name == "NotConnectedError":
        return "尚未连接设备，请先在「连接」面板选择串口并点击连接。"
    if name == "SendTimeoutError":
        return "命令发送超时，设备未响应。请检查连接是否正常、固件是否运行。"
    if name == "NackError":
        return f"设备拒绝命令：{e}"
    return str(e)


def _warn(parent: QWidget, title: str, msg: str) -> None:
    QMessageBox.warning(parent, title, msg)
