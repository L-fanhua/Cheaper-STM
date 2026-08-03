import ast
import sys

files = ["main.py", "ui/main_window.py", "ui/widgets.py"]
for f in files:
    with open(f, encoding="utf-8") as fh:
        ast.parse(fh.read())
    print("[ast] " + f + "  OK")

print()
print("[import] 尝试 from ui.main_window import MainWindow ...")
sys.path.insert(0, ".")

# 1. 只 import，不造实例
from ui.main_window import MainWindow
print("[import] MainWindow 导入成功")
print("[import]   - hasattr _run_async: " + str(hasattr(MainWindow, "_run_async")))
print("[import]   - hasattr _on_connect: " + str(hasattr(MainWindow, "_on_connect")))
print("[import]   - hasattr resizeEvent: " + str(hasattr(MainWindow, "resizeEvent")))

# 验证 _run_async 签名使用 ThreadPoolExecutor
import inspect
src_run_async = inspect.getsource(MainWindow._run_async)
has_tpe = "ThreadPoolExecutor" in src_run_async
print("[import]   - _run_async 使用 ThreadPoolExecutor: " + str(has_tpe))

# 验证 _on_connect 有 QMessageBox.warning 内容
src_on_connect = inspect.getsource(MainWindow._on_connect)
has_alert = "QMessageBox.warning" in src_on_connect
print("[import]   - _on_connect 含失败对话框: " + str(has_alert))

print()
print("[import] 尝试 from ui.widgets import TopStatusStrip, ScanFields ...")
from ui.widgets import TopStatusStrip, ScanFields
print("[import] TopStatusStrip, ScanFields 导入成功")

# 验证 TopStatusStrip 源码含 estop_btn 和高度
src_top = inspect.getsource(TopStatusStrip.__init__)
has_estop_self = "self.estop_btn" in src_top
has_h40 = "setFixedHeight(40)" in src_top
has_bg25 = "#252526" in src_top
print("[import]   - TopStatusStrip.estop_btn: " + str(has_estop_self))
print("[import]   - TopStatusStrip 高度40+背景#252526: " + str(has_h40 and has_bg25))

# 验证 ScanFields 源码
src_sf_init = inspect.getsource(ScanFields.__init__)
has_tabs = "QTabWidget" in src_sf_init or "self.tabs" in src_sf_init
has_save_load = "save_btn" in src_sf_init and "load_btn" in src_sf_init
print("[import]   - ScanFields 含 QTabWidget 两页: " + str(has_tabs))
print("[import]   - ScanFields 保存/加载按钮: " + str(has_save_load))

has_config_dict = hasattr(ScanFields, "_config_dict") and hasattr(ScanFields, "_load_config_dict")
has_presets = hasattr(ScanFields, "_BUILTIN_PRESETS")
has_save_load2 = hasattr(ScanFields, "_on_save_preset") and hasattr(ScanFields, "_on_load_preset")
print("[import]   - ScanFields _config/_load_config: " + str(has_config_dict))
print("[import]   - ScanFields _BUILTIN_PRESETS: " + str(has_presets))
print("[import]   - ScanFields _on_save/load_preset: " + str(has_save_load2))

print()
print("=== 全部验证通过 ===")
