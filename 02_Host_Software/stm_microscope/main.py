"""STM 上位机入口"""
from __future__ import annotations

import logging
import sys
import threading
import traceback

from PySide6.QtCore import Qt, Signal, QObject
from PySide6.QtWidgets import QApplication, QMessageBox

from ui.main_window import MainWindow


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler("stm.log", encoding="utf-8"),
        ],
    )


class _ExceptionHook(QObject):
    signal = Signal(str, str)

    def __init__(self):
        super().__init__()
        self.signal.connect(self._show_dialog, Qt.QueuedConnection)

    def _show_dialog(self, title: str, msg: str) -> None:
        try:
            QMessageBox.critical(None, title, msg)
        except Exception:
            print(f"[CRITICAL] {title}: {msg}", file=sys.stderr)

    def handle_exception(self, exc_type, exc_value, exc_tb):
        tb_str = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
        logging.getLogger("stm").exception("Uncaught exception", exc_info=(exc_type, exc_value, exc_tb))
        self.signal.emit(
            "未捕获异常（已阻止闪退）",
            f"{exc_type.__name__}: {exc_value}\n\n"
            f"详细堆栈已写入 stm.log，节选：\n" +
            "\n".join(tb_str.splitlines()[-15:])
        )

    def handle_thread_exception(self, args):
        self.handle_exception(args.exc_type, args.exc_value, args.exc_traceback)


def main() -> int:
    setup_logging()
    app = QApplication(sys.argv)
    app.setApplicationName("STM 上位机")
    app.setQuitOnLastWindowClosed(True)

    # 设置应用图标（任务栏 / Alt+Tab）
    import os
    icon_path = os.path.join(os.path.dirname(__file__), "ui", "LOGO.ico")
    if os.path.exists(icon_path):
        from PySide6.QtGui import QIcon
        app.setWindowIcon(QIcon(icon_path))

    hook = _ExceptionHook()
    sys.excepthook = hook.handle_exception
    if hasattr(threading, "excepthook"):
        threading.excepthook = hook.handle_thread_exception

    try:
        window = MainWindow()
    except Exception as e:
        logging.getLogger("stm").exception("MainWindow 创建失败")
        QMessageBox.critical(
            None, "启动失败",
            f"{type(e).__name__}: {e}\n\n"
            "详细日志见 stm.log。请检查依赖：pip install -r requirements.txt"
        )
        return 1

    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
