#!/usr/bin/env python3
"""
STM-FORESP32 通信测试脚本 (使用 STMHost 驱动, v1.1)

用法:
    python test_protocol.py --port COM3
    python test_protocol.py --port /dev/ttyACM0

演示:
  - 心跳监控 / 断连 / 自动重连
  - 固件版本查询 (GET_FW_VERSION)
  - 全部命令 ACK/NACK 验证
  - STATUS 事件回调
"""

import argparse
import time

from STMHost import STMHost, ConnectionState


def on_status(st):
    print(f"  [STATUS] mode={st['mode']} I={st['I']:.3f}nA z={st['z']:.3f}V "
          f"x={st['x']:.3f}V y={st['y']:.3f}V step={st['step_pos']} err=0x{st['err']:02X}")


def on_error(err):
    print(f"  [ERROR] code=0x{err['code']:02X} msg={err['msg']}")


def on_fw_version(ver):
    print(f"  [FW_VERSION] {ver}")


def on_state(st: ConnectionState):
    print(f"  [STATE] {st.value}")


def check(cmd_name: str, result: dict):
    if result["ok"]:
        print(f"  {cmd_name}: ACK")
    else:
        reason = result.get("reason", 0)
        tag = "DISCONNECTED" if result.get("disconnected") else "NACK"
        print(f"  {cmd_name}: {tag} reason=0x{reason:02X}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reconnect", action="store_true", help="禁用自动重连")
    args = ap.parse_args()

    host = STMHost(
        args.port, baud=args.baud,
        on_status=on_status, on_error=on_error,
        on_fw_version=on_fw_version, on_state=on_state,
        auto_reconnect=not args.no_reconnect,
        pong_timeout=3.0, ping_interval=1.0, reconnect_interval=1.0,
    )

    print(f"连接 {args.port} ...")
    if not host.connect(timeout=5.0):
        print("连接失败 (未收到 PONG)")
        if not args.no_reconnect:
            print("自动重连已启动, 等待 8 秒观察重连行为...")
            time.sleep(8)
        host.disconnect()
        return

    print("\n=== 查询固件版本 ===")
    host.get_fw_version()
    time.sleep(0.5)

    print("\n=== 命令测试 ===")
    check("SET_MODE(idle)", host.set_mode(0))
    check("SET_MODE(feedback)", host.set_mode(1))
    check("SET_MODE(approach)[expect NACK]", host.set_mode(3))

    check("SET_SETPOINT(1.0nA)", host.set_setpoint(1.0))
    check("SET_PID(0.1,0.01,0.001)", host.set_pid(0.1, 0.01, 0.001))
    check("SET_Z_LIMITS(-15,15)", host.set_z_limits(-15.0, 15.0))
    check("SET_PID_FREQ(1000)", host.set_pid_freq(1000))
    check("SET_OVERCURRENT(10nA)", host.set_overcurrent(10.0))

    check("SET_MODE(idle)", host.set_mode(0))
    check("MOVE_XYZ(0,0,0)", host.move_xyz(0.0, 0.0, 0.0))
    check("SET_Z(1.0)", host.set_z(1.0))

    check("SET_BIAS(2.5V)", host.set_bias(2.5))
    check("SET_SCAN_MODE(0=恒流)", host.set_scan_mode(0))
    check("STEP_MOTOR(100,forward,200sps)", host.step_motor(100, 0, 200))

    check("SET_CALIB(100M,10,10,10)", host.set_calib(100e6, 10, 10, 10))
    check("SAVE_CALIB", host.save_calib())
    check("LOAD_CALIB", host.load_calib())

    print("\n=== 主动查询状态 ===")
    host.get_status()
    time.sleep(0.5)

    print("\n=== 监听 3 秒 (STATUS 周期上报) ===")
    time.sleep(3)

    print("\n=== 断连测试 (拔线/关电源后观察自动重连) ===")
    print("(请在此期间断开 USB, 5 秒后重新插上以测试重连)")
    time.sleep(10)

    print("\n=== 主动断开 ===")
    host.disconnect()
    print("完成.")


if __name__ == "__main__":
    main()