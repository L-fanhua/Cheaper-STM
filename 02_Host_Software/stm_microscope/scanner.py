"""
STM 扫描数据引擎

职责：
- 接收 SCAN_ROW 流，组装成二维图像
- 提供实时图像数据 (numpy array) 供 UI 刷新
- 平面背景扣除 (leveling)
- 像素 ↔ 物理单位换算 (V → nm)
- 保存原始数据 / 导出为图片

线程模型：
- 在 transport 回调线程被调用 on_scan_row
- 图像数据用锁保护，UI 主线程读取拷贝
"""
from __future__ import annotations

import datetime
import json
import logging
import os
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from controller import Calibration, ScanParams
from protocol import ScanRow

logger = logging.getLogger(__name__)


@dataclass
class ScanImage:
    """
    一次扫描的图像数据。

    - I_image: 隧道电流图 (nA)，恒流模式下接近 setpoint
    - z_image: z 反馈电压图 (V)，恒流模式下的地形高度图 (主图)
    - 行优先，I_image[y, x]
    """
    params: ScanParams
    calib: Calibration
    nx: int
    ny: int
    I_image: np.ndarray = field(init=False)   # shape (ny, nx) float32
    z_image: np.ndarray = field(init=False)   # shape (ny, nx) float32
    rows_received: int = 0
    completed: bool = False
    start_time: str = ""
    end_time: str = ""

    def __post_init__(self) -> None:
        self.I_image = np.full((self.ny, self.nx), np.nan, dtype=np.float32)
        self.z_image = np.full((self.ny, self.nx), np.nan, dtype=np.float32)
        self.start_time = datetime.datetime.now().isoformat(timespec="seconds")

    # -----------------------------------------------------------------
    # 数据更新
    # -----------------------------------------------------------------

    def set_row(self, row: ScanRow) -> None:
        """写入一行数据"""
        if row.y_idx >= self.ny:
            logger.warning(f"row y_idx {row.y_idx} out of range ny={self.ny}")
            return
        if row.nx != self.nx:
            logger.warning(f"row nx {row.nx} != expected {self.nx}")
            return
        self.I_image[row.y_idx, :] = np.asarray(row.I_data, dtype=np.float32)
        self.z_image[row.y_idx, :] = np.asarray(row.z_data, dtype=np.float32)
        self.rows_received += 1

    def mark_completed(self) -> None:
        self.completed = True
        self.end_time = datetime.datetime.now().isoformat(timespec="seconds")

    # -----------------------------------------------------------------
    # 物理单位换算
    # -----------------------------------------------------------------

    def x_voltage_axis(self) -> np.ndarray:
        """x 轴电压坐标 (V)"""
        return np.linspace(
            self.params.cx_V - self.params.range_x_V / 2,
            self.params.cx_V + self.params.range_x_V / 2,
            self.nx,
            dtype=np.float32,
        )

    def y_voltage_axis(self) -> np.ndarray:
        """y 轴电压坐标 (V)"""
        return np.linspace(
            self.params.cy_V - self.params.range_y_V / 2,
            self.params.cy_V + self.params.range_y_V / 2,
            self.ny,
            dtype=np.float32,
        )

    def x_nm_axis(self) -> np.ndarray:
        """x 轴位移坐标 (nm)"""
        return self.x_voltage_axis() * self.calib.x_nmV

    def y_nm_axis(self) -> np.ndarray:
        """y 轴位移坐标 (nm)"""
        return self.y_voltage_axis() * self.calib.y_nmV

    def z_image_nm(self) -> np.ndarray:
        """z 图换算为 nm"""
        return self.z_image * self.calib.z_nmV

    def x_range_nm(self) -> float:
        return self.params.range_x_V * self.calib.x_nmV

    def y_range_nm(self) -> float:
        return self.params.range_y_V * self.calib.y_nmV


# ---------------------------------------------------------------------------
# 平面背景扣除
# ---------------------------------------------------------------------------

def plane_level(image: np.ndarray, mask: Optional[np.ndarray] = None) -> np.ndarray:
    """
    二维平面背景扣除 (常用 STM 后处理)。
    拟合 z = a*x + b*y + c，扣除该平面。

    - image: shape (ny, nx)
    - mask: 同形状 bool 数组，True 表示参与拟合的点 (默认全部)
    - 返回扣除平面后的 image (同样 shape)
    """
    ny, nx = image.shape
    yy, xx = np.mgrid[0:ny, 0:nx]
    xx = xx.astype(np.float64)
    yy = yy.astype(np.float64)
    data = image.astype(np.float64)
    if mask is None:
        mask = np.isfinite(data)
    else:
        mask = mask & np.isfinite(data)
    if mask.sum() < 3:
        return image.copy()
    # 构造方程 Ax = b，A = [x, y, 1]
    A = np.column_stack([
        xx[mask].ravel(),
        yy[mask].ravel(),
        np.ones(mask.sum()),
    ])
    b = data[mask].ravel()
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    plane = coef[0] * xx + coef[1] * yy + coef[2]
    return (data - plane).astype(np.float32)


def line_level(image: np.ndarray) -> np.ndarray:
    """
    逐行去背景 (每行减中位数)，对压电迟滞引起的行间偏移有效。
    """
    out = image.astype(np.float32).copy()
    finite_mask = np.isfinite(out)
    for y in range(out.shape[0]):
        row_mask = finite_mask[y]
        if row_mask.any():
            out[y] -= np.median(out[y][row_mask])
    return out


# ---------------------------------------------------------------------------
# 扫描引擎
# ---------------------------------------------------------------------------

class ScanEngine:
    """
    接收 ScanRow 流，管理当前扫描图像，线程安全。
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current: Optional[ScanImage] = None

    # -----------------------------------------------------------------
    # 控制接口
    # -----------------------------------------------------------------

    def start_scan(self, params: ScanParams, calib: Calibration) -> None:
        """开始新扫描：初始化空图像"""
        with self._lock:
            self._current = ScanImage(
                params=params,
                calib=calib,
                nx=params.nx,
                ny=params.ny,
            )

    def on_row(self, row: ScanRow) -> None:
        """transport 回调：写入一行"""
        with self._lock:
            if self._current is None:
                logger.warning("ScanRow received but no scan in progress")
                return
            self._current.set_row(row)

    def on_end(self, total_points: int) -> None:
        """transport 回调：扫描完成"""
        with self._lock:
            if self._current is not None:
                self._current.mark_completed()

    def abort(self) -> None:
        """中止扫描 (保留已收数据)"""
        with self._lock:
            if self._current is not None and not self._current.completed:
                self._current.mark_completed()

    # -----------------------------------------------------------------
    # 查询
    # -----------------------------------------------------------------

    @property
    def current(self) -> Optional[ScanImage]:
        with self._lock:
            return self._current

    def snapshot(self) -> Optional[ScanImage]:
        """获取当前图像的引用 (UI 线程读取时锁内拷贝 numpy 数据即可)"""
        with self._lock:
            return self._current

    def progress(self) -> float:
        """返回扫描进度 0~1"""
        with self._lock:
            if self._current is None or self._current.ny == 0:
                return 0.0
            return self._current.rows_received / self._current.ny


# ---------------------------------------------------------------------------
# 数据保存
# ---------------------------------------------------------------------------

def save_scan(image: ScanImage, directory: str = "data") -> str:
    """
    保存扫描数据。
    - .npz: 原始数据 (含 params/calib/图像)
    - .json: 元数据
    返回保存的 .npz 文件路径。
    """
    os.makedirs(directory, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(directory, f"scan_{ts}")

    # 元数据
    meta = {
        "timestamp": ts,
        "start_time": image.start_time,
        "end_time": image.end_time,
        "completed": image.completed,
        "nx": image.nx,
        "ny": image.ny,
        "rows_received": image.rows_received,
        "params": {
            "cx_V": image.params.cx_V,
            "cy_V": image.params.cy_V,
            "range_x_V": image.params.range_x_V,
            "range_y_V": image.params.range_y_V,
            "nx": image.params.nx,
            "ny": image.params.ny,
            "speed_pps": image.params.speed_pps,
            "direction": int(image.params.direction),
            "settle_ms": image.params.settle_ms,
            "overshoot_V": image.params.overshoot_V,
        },
        "calib": {
            "tia_R": image.calib.tia_R,
            "x_nmV": image.calib.x_nmV,
            "y_nmV": image.calib.y_nmV,
            "z_nmV": image.calib.z_nmV,
        },
    }
    with open(f"{base}.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # 原始数据
    np.savez(
        f"{base}.npz",
        I_image=image.I_image,
        z_image=image.z_image,
        x_axis_V=image.x_voltage_axis(),
        y_axis_V=image.y_voltage_axis(),
    )
    return f"{base}.npz"


def load_scan(npz_path: str, json_path: Optional[str] = None) -> ScanImage:
    """加载 .npz 文件重建 ScanImage"""
    if json_path is None:
        json_path = npz_path.replace(".npz", ".json")
    with open(json_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    data = np.load(npz_path)
    params = ScanParams(
        cx_V=meta["params"]["cx_V"],
        cy_V=meta["params"]["cy_V"],
        range_x_V=meta["params"]["range_x_V"],
        range_y_V=meta["params"]["range_y_V"],
        nx=meta["params"]["nx"],
        ny=meta["params"]["ny"],
        speed_pps=meta["params"]["speed_pps"],
        direction=ScanDirection(meta["params"]["direction"]),
        settle_ms=meta["params"]["settle_ms"],
        overshoot_V=meta["params"]["overshoot_V"],
    )
    calib = Calibration(
        tia_R=meta["calib"]["tia_R"],
        x_nmV=meta["calib"]["x_nmV"],
        y_nmV=meta["calib"]["y_nmV"],
        z_nmV=meta["calib"]["z_nmV"],
    )
    img = ScanImage(params=params, calib=calib, nx=params.nx, ny=params.ny)
    img.I_image = data["I_image"]
    img.z_image = data["z_image"]
    img.rows_received = int(np.isfinite(img.I_image).sum()) // img.nx
    img.completed = meta["completed"]
    img.start_time = meta["start_time"]
    img.end_time = meta["end_time"]
    return img


# 延迟导入避免循环依赖
from protocol import ScanDirection  # noqa: E402
