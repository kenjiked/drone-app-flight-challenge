#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡回デモ 背骨(walking skeleton) — 中心座標をもとに四角の巡回ミッションを作り、SITLで飛ばす。

流れ:
  中心座標 → 四角(外周)の巡回ミッション生成 → SITLへアップロード → AUTOで巡回飛行 → RTL(帰還)

これは「薄く一本、端から端まで通す」最初のピース(design-decisions.md D14)。
まだ地図もジオコーディングもUIも無い。まず「座標→実際に飛ぶ」を成立させるのが目的。
のちに前段へ「住所→座標」を、後段へ「安全チェック/飛行中監視」を足していく。

ベース: dronekit-python examples/mission_basic/mission_basic.py の実績ある流れを流用。

使い方(例):
  # SITL(sim_vehicle.py 等)を起動しておき、その出力先に接続する
  python3 src/patrol_spine.py --connect udp:127.0.0.1:14550
  # 中心座標を指定しない場合は、機体の現在地(SITLのホーム)を中心にする
  python3 src/patrol_spine.py --connect udp:127.0.0.1:14550 --lat 35.681 --lon 139.767
"""
from __future__ import print_function

import argparse
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil

DEFAULT_CONNECT = "udp:127.0.0.1:14550"
DEFAULT_SIZE_M = 50.0   # 中心から各辺までの距離(m)。四角の一辺は約 2*SIZE。
DEFAULT_ALT_M = 15.0    # 巡回する高度(m)


def get_location_metres(original_location, dNorth, dEast):
    """original_location から北へ dNorth[m]、東へ dEast[m] ずらした地点を返す。
    小距離(1km内で10m程度)なら十分な精度。極付近では不正確。"""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """2地点間の地表距離(m)の近似。"""
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def distance_to_current_waypoint(vehicle):
    """現在向かっているウェイポイントまでの距離(m)。ホーム(0)ならNone。"""
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint - 1]  # commands は 0 始まり
    lat, lon, alt = missionitem.x, missionitem.y, missionitem.z
    target = LocationGlobalRelative(lat, lon, alt)
    return get_distance_metres(vehicle.location.global_frame, target)


def build_patrol_mission(vehicle, center, size_m, alt_m):
    """center を中心に、一辺 2*size_m の四角(外周)を巡回するミッションを組んでアップロードする。
    返り値: 巡回ウェイポイントの数(角の数)。"""
    cmds = vehicle.commands
    print(" 既存ミッションをクリア")
    cmds.download()
    cmds.wait_ready()
    cmds.clear()

    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    # 1) 離陸(TAKEOFF): alt_m まで上がる
    cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, alt_m))

    # 2) 四角の4つの角(外周をなぞる)
    corners = [
        get_location_metres(center,  size_m, -size_m),
        get_location_metres(center,  size_m,  size_m),
        get_location_metres(center, -size_m,  size_m),
        get_location_metres(center, -size_m, -size_m),
    ]
    for c in corners:
        cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, c.lat, c.lon, alt_m))

    # 3) 最初の角に戻って一周を閉じる + 終了検知用マーカー
    close = corners[0]
    cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0, close.lat, close.lon, alt_m))

    print(" ミッションをアップロード")
    cmds.upload()
    # home(seq0) + takeoff(seq1) + 角4(seq2..5) + 閉じ(seq6) → 最後の seq は 6
    return len(corners)


def build_polygon_mission(vehicle, corners_latlng, alt_m):
    """任意の多角形（頂点 [[lat,lon],...]）の外周を巡回するミッションを組んでアップロードする。
    四角(build_patrol_mission)の一般化。地図をなぞって作ったルート(UI C)用。
    返り値: 巡回ウェイポイント数（頂点数）。"""
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()

    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, alt_m))
    for lat, lon in corners_latlng:
        cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, lat, lon, alt_m))
    # 最初の頂点に戻って一周を閉じる
    lat0, lon0 = corners_latlng[0]
    cmds.add(Command(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0, lat0, lon0, alt_m))
    cmds.upload()
    return len(corners_latlng)


def arm_and_takeoff(vehicle, target_alt):
    """アーム前チェックを待ってからアーム→離陸(target_alt まで)。"""
    print("離陸前チェック中...")
    while not vehicle.is_armable:
        print(" 機体の初期化待ち...")
        time.sleep(1)

    print("アーム(GUIDEDモード)")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" アーム待ち...")
        time.sleep(1)

    print("離陸！")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(" 高度: %.1f m" % (alt or 0.0))
        if alt is not None and alt >= target_alt * 0.95:
            print("目標高度に到達")
            break
        time.sleep(1)


def wait_until_ready(vehicle, timeout=120):
    """GPS/EKF が整い is_armable になるまで待つ。
    現在地が有効になる前にミッションを作ると中心が(0,0)になるため、必ず先に待つ。"""
    print("機体の準備(GPS/EKF)を待機...")
    waited = 0
    while not vehicle.is_armable:
        fix = vehicle.gps_0.fix_type if vehicle.gps_0 else "?"
        print(" 初期化待ち... GPS fix=%s" % fix)
        time.sleep(1)
        waited += 1
        if waited >= timeout:
            raise RuntimeError("準備がtimeout。GPS/EKFが整いませんでした")
    print("準備OK (armable)")


def run_patrol(vehicle, size_m, alt_m, center=None):
    """一連の巡回を実行する。center 未指定なら現在地(ホーム)を中心にする。"""
    # GPSが整う前に現在地を読むと中心が(0,0)になるため、先に準備を待つ
    wait_until_ready(vehicle)

    if center is None:
        center = vehicle.location.global_frame
        if center is None or center.lat is None or (abs(center.lat) < 1e-6 and abs(center.lon) < 1e-6):
            raise RuntimeError("現在地が無効(0,0)。GPS未取得の可能性。--lat/--lon 指定も検討")
        print("中心 = 機体の現在地 (lat=%.6f, lon=%.6f)" % (center.lat, center.lon))
    else:
        print("中心 = 指定座標 (lat=%.6f, lon=%.6f)" % (center.lat, center.lon))

    print("巡回ミッションを作成 (一辺 約%.0fm, 高度 %.0fm)" % (size_m * 2, alt_m))
    num_corners = build_patrol_mission(vehicle, center, size_m, alt_m)
    last_seq = 1 + num_corners + 1  # takeoff(1) + 角(num) + 閉じ(1)

    arm_and_takeoff(vehicle, alt_m)

    print("巡回開始 (AUTOモード)")
    vehicle.commands.next = 0
    vehicle.mode = VehicleMode("AUTO")

    while True:
        nextwp = vehicle.commands.next
        dist = distance_to_current_waypoint(vehicle)
        # 角の何個目か(離陸=seq1 を除いた進捗)を平易に表示
        corner_idx = max(0, nextwp - 1)
        if dist is not None:
            print(" 巡回中 %d/%d地点  次の地点まで %.0fm  [姿勢OK・範囲内]"
                  % (min(corner_idx, num_corners), num_corners, dist))
        if nextwp >= last_seq:
            print("最終地点に到達。一周完了")
            break
        time.sleep(1)

    print("帰還(RTL)")
    vehicle.mode = VehicleMode("RTL")


def main():
    parser = argparse.ArgumentParser(description="巡回デモ 背骨: 座標→四角巡回→SITLで飛行")
    parser.add_argument("--connect", default=DEFAULT_CONNECT,
                        help="接続先 (例: udp:127.0.0.1:14550, tcp:127.0.0.1:5760)")
    parser.add_argument("--lat", type=float, default=None, help="巡回中心の緯度(省略時は現在地)")
    parser.add_argument("--lon", type=float, default=None, help="巡回中心の経度(省略時は現在地)")
    parser.add_argument("--size", type=float, default=DEFAULT_SIZE_M, help="中心から辺までの距離(m)")
    parser.add_argument("--alt", type=float, default=DEFAULT_ALT_M, help="巡回高度(m)")
    args = parser.parse_args()

    center = None
    if args.lat is not None and args.lon is not None:
        center = LocationGlobal(args.lat, args.lon, args.alt)

    print("接続中: %s" % args.connect)
    vehicle = connect(args.connect, wait_ready=True)
    try:
        run_patrol(vehicle, args.size, args.alt, center=center)
    finally:
        print("接続を閉じる")
        vehicle.close()


if __name__ == "__main__":
    main()
