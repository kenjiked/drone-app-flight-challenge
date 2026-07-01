#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""住所を入れるだけで、その場所を巡回する —— 非エンジニア向けの1コマンド体験。

中で以下を自動で行う（ユーザーは住所を言うだけ）:
  1. 住所 → 座標           (geocode.py)
  2. その座標をホームにSITLを起動
  3. 接続して巡回ミッションを飛ばす (patrol_spine.py)
  4. 片付け（SITL停止）

使い方:
  python3 src/plan_and_fly.py "名古屋城"
  python3 src/plan_and_fly.py            # 住所を対話で聞く

注意: これはデモ用。飛行中の「姿勢OK・範囲内」表示はまだ固定文字(張りぼて)。
      本物の安全監視(③)は次の実装で置き換える。
"""
from __future__ import print_function

import os
import signal
import socket
import subprocess
import sys
import time

# 同じ src/ 内の自作モジュールを使う（このスクリプトのある場所が import パスに入る）
from geocode import geocode
from patrol_spine import run_patrol
from dronekit import connect

ARDUPILOT_DIR = os.path.expanduser("~/GitHub/ardupilot")
SIM_VEHICLE = os.path.join(ARDUPILOT_DIR, "Tools/autotest/sim_vehicle.py")
CONNECT_STR = "tcp:127.0.0.1:5760"
SIM_HOST, SIM_PORT = "127.0.0.1", 5760

PATROL_SIZE_M = 40.0   # 中心から辺までの距離(m)
PATROL_ALT_M = 15.0    # 巡回高度(m)


def wait_for_port(host, port, timeout=90):
    """SITL が接続を受け付ける(ポートが開く)まで待つ。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False


def launch_sitl(lat, lon):
    """指定座標をホームに ArduCopter SITL を起動し、プロセスを返す。"""
    cmd = [sys.executable, SIM_VEHICLE, "-v", "ArduCopter", "--no-mavproxy",
           "--custom-location=%f,%f,0,0" % (lat, lon), "-w"]
    return subprocess.Popen(
        cmd, cwd=ARDUPILOT_DIR,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,  # 新しいプロセスグループ = まとめて止められる
    )


def stop_sitl(proc):
    """SITL とその子プロセス(arducopter 等)をまとめて停止する。"""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass
    # 取りこぼし対策（xterm 経由で起動される arducopter 等）
    for pat in ("build/sitl/bin/arducopter", "sim_vehicle.py"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    address = " ".join(sys.argv[1:]).strip()
    if not address:
        try:
            address = input("どこを見回りますか？（住所や地名）: ").strip()
        except EOFError:
            address = ""
    if not address:
        print("住所が入力されませんでした。")
        return 2

    print("1) 住所を座標に変換します…")
    lat, lon, source = geocode(address)
    print("   「%s」→ 緯度 %.6f, 経度 %.6f （%s）" % (address, lat, lon, source))

    print("2) その場所でシミュレータを起動します…（30秒ほどかかります）")
    sitl = launch_sitl(lat, lon)
    try:
        if not wait_for_port(SIM_HOST, SIM_PORT):
            print("   シミュレータの起動に失敗しました。")
            return 1
        print("   起動OK")

        print("3) 接続して巡回を開始します…")
        vehicle = connect(CONNECT_STR, wait_ready=True)
        try:
            run_patrol(vehicle, PATROL_SIZE_M, PATROL_ALT_M)  # center未指定=現在地(=住所の場所)
        finally:
            vehicle.close()

        print("4) 完了しました。お疲れさまでした。")
        return 0
    finally:
        print("   シミュレータを片付けます…")
        stop_sitl(sitl)


if __name__ == "__main__":
    sys.exit(main())
