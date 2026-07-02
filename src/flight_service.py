#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飛行サービス層 — WebのUIとArduPilot(SITL)を繋ぐ「裏方」。

役割:
  - 住所→座標(geocode)、離陸前の安全チェック(precheck)、巡回飛行の実行(start)を
    まとめて提供する。
  - 飛行は別スレッドで走らせ、実テレメトリ(モード/高度/巡回進捗/姿勢/範囲/電池)を
    共有状態(self._state)に書き込む。UI(server.py)はこれを /api/status で読むだけ。

これにより、UIの「張りぼて(固定文字)」を、ArduPilotの実データに置き換える。
既存の巡回ロジック(patrol_spine.py)・住所変換(geocode.py)・SITL起動(plan_and_fly.py)を再利用する。

デモ構成(design-decisions.md D14 walking skeleton):
  ブラウザUI → server.py(HTTP) → FlightManager(このファイル) → dronekit/MAVLink → ArduPilot SITL

安全監視(⑤)は「親切版=Python側の監視」。ArduPilotフライトコード側の深い拡張(③の芯)は次段階。
"""
from __future__ import print_function

import math
import threading
import time
import traceback

from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil

# 同じ src/ 内の自作モジュールを再利用する
import geo_safety
from geocode import geocode
from patrol_spine import (
    build_patrol_mission,
    distance_to_current_waypoint,
    get_distance_metres,
    get_location_metres,
)
from plan_and_fly import launch_sitl, stop_sitl, wait_for_port, SIM_HOST, SIM_PORT, CONNECT_STR

# --- 飛行パラメータの既定 ---
DEFAULT_SIDE_M = 160.0   # 巡回の一辺(m)。UIのスライダ既定と合わせる。
DEFAULT_ALT_M = 25.0     # 巡回高度(m)。
CRUISE_SPEED_MPS = 5.0   # 見積り用の水平巡航速度(m/s)。
CLIMB_SPEED_MPS = 2.5    # 見積り用の上昇速度(m/s)。
ENDURANCE_S = 720.0      # 見積り用の電池による総飛行可能時間(s) ≒ 12分。
NUM_CORNERS = 4          # 四角の角の数(巡回地点数)。


class FlightManager(object):
    """1機分の飛行状態を持ち、飛行を別スレッドで実行する。

    UIからは start()/stop()/precheck()/snapshot() を呼ぶだけ。
    スレッド間で共有する状態(self._state)は self._lock で保護する。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._stop_flag = False
        self._sitl = None
        self._vehicle = None
        self._state = self._initial_state()

    # ------------------------------------------------------------------ 状態
    def _initial_state(self):
        return {
            "phase": "idle",        # idle/geocoding/launching/connecting/prearm/takeoff/patrol/rtl/done/error
            "message": "待機中",
            "address": None,
            "lat": None,          # 巡回中心(=住所)の緯度
            "lon": None,          # 巡回中心(=住所)の経度
            "veh_lat": None,      # 機体の現在緯度(飛行中に更新)
            "veh_lon": None,      # 機体の現在経度(飛行中に更新)
            "corners": None,      # 巡回ルート(四角)の角の[lat,lon]リスト
            "geocode_source": None,
            "side_m": DEFAULT_SIDE_M,
            "alt_m": DEFAULT_ALT_M,
            "connected": False,
            "mode": None,
            "armed": False,
            "alt": 0.0,             # 現在高度(m)
            "battery": None,        # 電池残量(%) or None
            "wp_index": 0,          # 今向かっている巡回地点(1..NUM_CORNERS)
            "wp_total": NUM_CORNERS,
            "wp_distance": None,    # 次の地点までの距離(m)
            "attitude_ok": None,    # 姿勢が安全範囲内か
            "in_range": None,       # 巡回範囲内にいるか
            "battery_ok": None,     # 電池が安全か
            "error": None,
            "started_at": None,
            "finished_at": None,
        }

    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def snapshot(self):
        """UIに返す状態のコピー。"""
        with self._lock:
            return dict(self._state)

    def is_running(self):
        with self._lock:
            phase = self._state["phase"]
        return phase not in ("idle", "done", "error")

    # ------------------------------------------------- ①住所→座標(単発, UI用)
    def geocode(self, address):
        """住所→(lat,lon,source)。UIの「場所を探す」ボタン用。飛行はしない。"""
        lat, lon, source = geocode(address)
        return {"address": address, "lat": lat, "lon": lon, "source": source}

    # -------------------------------------------------- ③離陸前 安全チェック
    def precheck(self, side_m, alt_m, lat=None, lon=None):
        """離陸前の安全チェック(親切版)。実際の数値見積りを返す。

        - 電池: 巡回の総距離から推定飛行時間を出し、電池の総飛行可能時間と比べる。
        - 高さ: 法令・安全の上限内か。
        - 範囲↔フェンス整合(A): 巡回範囲の角が機体ジオフェンス内か（外部データ不要の実計算）。
        - 空港・空域近接(B): 最寄り空港との距離（同梱座標＋haversine）。※警告(飛行は止めない)
        - 飛行禁止ゾーン(D): サンプル禁止ポリゴンに掛かるか（点in多角形）。※警告
        各チェックは {ok, label, detail, level} を返す。level='block' のみが飛行可否(ok)を左右し、
        'warn' は注意喚起のみ（デモの happy-path を壊さないため）。lat/lon が無ければ位置系は省く。
        設計: docs/safety-design.md §8。
        """
        side_m = float(side_m)
        alt_m = float(alt_m)
        half = side_m / 2.0

        # 概算の飛行経路長: 中心→角(対角) + 四辺 + 閉じ + 帰還(対角)
        diag = half * math.sqrt(2)
        horizontal = diag + 4 * side_m + side_m + diag
        est_time = horizontal / CRUISE_SPEED_MPS + alt_m / CLIMB_SPEED_MPS * 2

        # 電池: 推定飛行時間 < 総飛行可能時間の60% なら安全(往復＋余裕)
        batt_ok = est_time < ENDURANCE_S * 0.6

        # 範囲↔フェンス整合(A): 角(=半辺*√2) が Lua しきい値(FENCE_RADIUS*0.9) 以内か
        corner_m = geo_safety.corner_distance_m(side_m)
        fence_lim = geo_safety.fence_limit_m()
        fence_ok = corner_m <= fence_lim

        checks = {
            "battery": {
                "ok": batt_ok, "level": "block",
                "label": "電池は足ります（往復に十分）",
                "detail": "推定飛行 約%d分 / 電池 約%d分（%d%%使用）" % (
                    round(est_time / 60), round(ENDURANCE_S / 60),
                    round(est_time / ENDURANCE_S * 100)),
            },
            "altitude": {
                "ok": 10.0 <= alt_m <= 60.0, "level": "block",
                "label": "高さは安全な範囲です",
                "detail": "高度 %dm（安全範囲 10〜60m）" % round(alt_m),
            },
            "fence": {
                "ok": fence_ok, "level": "block",
                "label": "範囲は機体フェンス内です",
                "detail": ("巡回の角 %dm ≦ フェンス %dm（RTL手前）" % (round(corner_m), round(fence_lim)))
                    if fence_ok else
                    ("巡回の角 %dm ＞ フェンス %dm → 飛行中に自動RTLの恐れ。範囲を狭めてください"
                     % (round(corner_m), round(fence_lim))),
            },
        }

        # 位置ベースの評価（住所を確定していれば実施）
        order = ["battery", "altitude", "fence"]
        if lat is not None and lon is not None:
            lat = float(lat); lon = float(lon)
            near = geo_safety.nearest_airport(lat, lon)
            if near:
                aname, adist = near
                airport_ok = adist >= geo_safety.AIRPORT_WARN_M
                checks["airport"] = {
                    "ok": airport_ok, "level": "warn",
                    "label": "空港・空域から離れています",
                    "detail": ("最寄り %s 約%.1fkm（%dkm圏外）" % (aname, adist / 1000.0,
                                round(geo_safety.AIRPORT_WARN_M / 1000)))
                        if airport_ok else
                        ("最寄り %s 約%.1fkm → 空港周辺は飛行に許可が要る場合があります"
                         % (aname, adist / 1000.0)),
                }
                order.append("airport")

            points = [[lat, lon]] + geo_safety.square_corners(lat, lon, side_m)
            hit = geo_safety.zones_hit(points)
            checks["nofly"] = {
                "ok": len(hit) == 0, "level": "warn",
                "label": "飛行禁止ゾーンに掛かりません",
                "detail": "サンプル禁止ゾーンの外です（デモ用データ）" if not hit
                    else ("禁止ゾーンに接触: %s（デモ用データ）" % "、".join(hit)),
            }
            order.append("nofly")

        # 飛行可否は block レベルのみで判定（warn は止めない）
        all_ok = all(c["ok"] for c in checks.values() if c.get("level") == "block")
        return {"ok": all_ok, "checks": checks, "order": order,
                "est_time_s": round(est_time), "est_distance_m": round(horizontal)}

    # -------------------------------------------------------- ④実行(開始)
    def start(self, address=None, lat=None, lon=None, side_m=DEFAULT_SIDE_M,
              alt_m=DEFAULT_ALT_M, connect_str=None):
        """巡回飛行を開始する。すぐ返り、実処理は別スレッドで進む。

        connect_str を渡すと、既に起動済みの SITL に接続する(プレゼンで --map を見せたい時)。
        省略時は、住所の座標をホームにして SITL を自動起動する(1コマンド体験)。
        """
        if self.is_running():
            raise RuntimeError("すでに飛行中です")

        self._stop_flag = False
        self._state = self._initial_state()
        self._set(side_m=float(side_m), alt_m=float(alt_m), address=address,
                  lat=lat, lon=lon, phase="geocoding", message="準備中…",
                  started_at=time.time())

        self._thread = threading.Thread(
            target=self._run,
            args=(address, lat, lon, float(side_m), float(alt_m), connect_str),
            daemon=True)
        self._thread.start()
        return self.snapshot()

    def stop(self):
        """飛行を止めて帰還(RTL)させ、片付ける。"""
        self._stop_flag = True
        v = self._vehicle
        if v is not None:
            try:
                v.mode = VehicleMode("RTL")
            except Exception:
                pass
        return self.snapshot()

    # ------------------------------------------------------ 実処理(別スレッド)
    def _run(self, address, lat, lon, side_m, alt_m, connect_str):
        half = side_m / 2.0
        try:
            # 1) 住所→座標(UIが座標を渡していなければ変換)
            if lat is None or lon is None:
                self._set(phase="geocoding", message="住所を座標に変換中…")
                lat, lon, source = geocode(address or "")
                self._set(lat=lat, lon=lon, geocode_source=source)
            else:
                self._set(geocode_source="ui")

            # 2) SITL接続(既存に接続 or 自動起動)
            target = connect_str
            if target:
                self._set(phase="connecting", message="ArduPilotに接続中…")
            else:
                self._set(phase="launching", message="シミュレータを起動中…（約30秒）")
                self._sitl = launch_sitl(lat, lon)
                if not wait_for_port(SIM_HOST, SIM_PORT):
                    raise RuntimeError("シミュレータの起動に失敗しました")
                target = CONNECT_STR
                self._set(phase="connecting", message="ArduPilotに接続中…")

            self._vehicle = connect(target, wait_ready=True)
            self._set(connected=True)

            center = LocationGlobal(lat, lon, alt_m)
            self._fly(self._vehicle, center, half, alt_m)

            if not self._stop_flag:
                self._set(phase="done", message="みまわり完了。無事に戻りました。",
                          finished_at=time.time())
            else:
                # ユーザーが途中で中止 → 帰還させて終了(終端フェーズにする)
                self._set(phase="done", message="中止しました。ドローンは帰還しました。",
                          finished_at=time.time())
        except Exception as e:
            self._set(phase="error",
                      message="エラー: %s" % e,
                      error="%s\n%s" % (e, traceback.format_exc()),
                      finished_at=time.time())
        finally:
            self._cleanup()

    def _fly(self, vehicle, center, half, alt_m):
        """離陸→AUTO巡回→帰還。毎秒テレメトリを状態へ反映する。"""
        # 準備(GPS/EKF)待ち
        self._set(phase="prearm", message="機体の準備(GPS/EKF)を待機中…")
        t0 = time.time()
        while not vehicle.is_armable:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            if time.time() - t0 > 120:
                raise RuntimeError("準備がtimeout。GPS/EKFが整いませんでした")
            time.sleep(1)

        # ミッション生成(四角の外周) + アップロード
        self._set(message="巡回ルートをアップロード中…")
        num_corners = build_patrol_mission(vehicle, center, half, alt_m)

        # 地図描画用に、実際の巡回ルート(四角の角)の緯度経度を状態へ入れる
        # ※ build_patrol_mission と同じ順序・同じ座標にする
        corner_locs = [
            get_location_metres(center,  half, -half),
            get_location_metres(center,  half,  half),
            get_location_metres(center, -half,  half),
            get_location_metres(center, -half, -half),
        ]
        self._set(corners=[[c.lat, c.lon] for c in corner_locs])

        # アーム→離陸
        self._set(phase="takeoff", message="離陸中…")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            time.sleep(1)
        vehicle.simple_takeoff(alt_m)
        while True:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            cur = vehicle.location.global_relative_frame.alt or 0.0
            if cur >= alt_m * 0.95:
                break
            time.sleep(1)

        # AUTOで巡回
        self._set(phase="patrol", message="みまわり中…")
        vehicle.commands.next = 0
        vehicle.mode = VehicleMode("AUTO")
        last_seq = 1 + num_corners + 1   # takeoff(1) + 角(num) + 閉じ(1)
        while True:
            if self._stop_flag:
                break
            self._push_telemetry(vehicle, center, half)
            if vehicle.commands.next >= last_seq:
                break
            time.sleep(1)

        # 帰還(RTL)
        self._set(phase="rtl", message="帰還中(RTL)…")
        vehicle.mode = VehicleMode("RTL")
        t0 = time.time()
        while time.time() - t0 < 120:
            self._push_telemetry(vehicle, center, half)
            cur = vehicle.location.global_relative_frame.alt or 0.0
            if not vehicle.armed or cur < 1.5:
                break
            time.sleep(1)

    def _push_telemetry(self, vehicle, center, half):
        """機体の実データを読み、状態と安全判定を更新する。"""
        try:
            att = vehicle.attitude
            roll = att.roll if att else 0.0
            pitch = att.pitch if att else 0.0
            attitude_ok = abs(roll) < 0.52 and abs(pitch) < 0.52  # ±30度以内

            loc = vehicle.location.global_relative_frame
            alt = loc.alt if loc and loc.alt is not None else 0.0

            # 範囲チェック + 地図用の機体現在地
            gframe = vehicle.location.global_frame
            in_range = None
            veh_lat = veh_lon = None
            if gframe and gframe.lat is not None:
                dist = get_distance_metres(gframe, center)
                in_range = dist <= half * math.sqrt(2) * 1.3
                veh_lat, veh_lon = gframe.lat, gframe.lon

            level = vehicle.battery.level if vehicle.battery else None
            battery_ok = (level is None) or (level > 20)

            wp_next = vehicle.commands.next
            wp_index = max(0, min(NUM_CORNERS, wp_next - 1))
            wp_dist = distance_to_current_waypoint(vehicle)

            self._set(
                mode=vehicle.mode.name if vehicle.mode else None,
                armed=bool(vehicle.armed),
                alt=round(alt, 1),
                battery=level,
                attitude_ok=attitude_ok,
                in_range=in_range,
                battery_ok=battery_ok,
                wp_index=wp_index,
                wp_distance=round(wp_dist) if wp_dist is not None else None,
                veh_lat=veh_lat,
                veh_lon=veh_lon,
            )
        except Exception:
            # テレメトリ取得失敗は致命的ではない。次の周回で回復を試みる。
            pass

    def _cleanup(self):
        v = self._vehicle
        self._vehicle = None
        if v is not None:
            try:
                v.close()
            except Exception:
                pass
        self._set(connected=False)
        sitl = self._sitl
        self._sitl = None
        if sitl is not None:
            stop_sitl(sitl)


# モジュール共有の単一インスタンス(デモは1機)
manager = FlightManager()
