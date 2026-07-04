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
    build_polygon_mission,
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


def _clean_corners(corners):
    """UI から来た多角形頂点を検証。[[lat,lon],...] 3点以上なら [[float,float],...]、他は None。"""
    if not corners:
        return None
    try:
        pts = [[float(p[0]), float(p[1])] for p in corners]
    except (TypeError, ValueError, IndexError):
        return None
    return pts if len(pts) >= 3 else None


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
            "home_lat": None,     # 離発着地点(ホーム)の緯度
            "home_lon": None,     # 離発着地点(ホーム)の経度
            "veh_lat": None,      # 機体の現在緯度(飛行中に更新)
            "veh_lon": None,      # 機体の現在経度(飛行中に更新)
            "corners": None,      # 巡回ルート(四角)の角の[lat,lon]リスト
            "geocode_source": None,
            # --- ArduPilot 内部の可視化（飛行中⑤で表示） ---
            "ekf_ok": None,       # EKF(位置推定)が健全か
            "gps_fix": None,      # GPS fix type (3=3D fix 等)
            "gps_sats": None,     # 捕捉衛星数
            "armable": None,      # 離陸可能(全pre-armクリア)か
            "sys_status": None,   # 機体システム状態(STANDBY/ACTIVE 等)
            "events": [],         # ArduPilotからのメッセージ(STATUSTEXT)ログ
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
            "max_alt": 0.0,        # 飛行中の最高高度(m)（飛行日誌用）
            "flight_log": None,    # 飛行終了時に自動生成する飛行日誌テキスト
        }

    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def snapshot(self):
        """UIに返す状態のコピー。"""
        with self._lock:
            return dict(self._state)

    def _add_event(self, severity, text):
        """ArduPilotからのメッセージ(STATUSTEXT等)をログに積む。直近60件を保持。"""
        with self._lock:
            evs = self._state.get("events") or []
            evs = evs[-59:] + [{"sev": int(severity), "text": str(text),
                                "t": round(time.time() - (self._state.get("started_at") or time.time()), 1)}]
            self._state["events"] = evs

    def is_running(self):
        with self._lock:
            phase = self._state["phase"]
        return phase not in ("idle", "done", "error")

    def _build_flight_log(self, result):
        """飛行日誌（テキスト）を自動生成する。航空法で作成・保存が求められる記録の下書き。"""
        import datetime
        s = self.snapshot()

        def fmt(ts):
            if not ts:
                return "-"
            return datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")

        dur = "-"
        if s.get("started_at") and s.get("finished_at"):
            d = int(s["finished_at"] - s["started_at"])
            dur = "%d分%02d秒" % (d // 60, d % 60)

        lines = [
            "みまわり自作ドローン — 飛行記録（飛行日誌）",
            "=" * 44,
            "飛行日時    : %s 〜 %s（%s）" % (fmt(s.get("started_at")), fmt(s.get("finished_at")), dur),
            "飛行場所    : %s（%.6f, %.6f）" % (s.get("address") or "-",
                                                s.get("lat") or 0.0, s.get("lon") or 0.0),
            "離発着地点  : %.6f, %.6f" % (s.get("home_lat") or 0.0, s.get("home_lon") or 0.0),
            "ルート      : 外周みまわり %d地点（設定: 一辺%dm・高度%dm）" % (
                s.get("wp_total") or 0, round(s.get("side_m") or 0), round(s.get("alt_m") or 0)),
            "最高高度    : %.1fm" % (s.get("max_alt") or 0.0),
            "結果        : %s" % result,
            "機体        : ArduCopter（SITLシミュレーション）",
            "",
            "経過（機体からの主なメッセージ）:",
        ]
        for e in s.get("events") or []:
            lines.append("  +%ss  %s" % (e.get("t"), e.get("text")))
        lines += [
            "",
            "※実機の飛行では、飛行日誌の作成・保存が航空法で義務付けられています。",
            "  この記録はその下書きとして自動生成されました。",
        ]
        return "\n".join(lines)

    # ------------------------------------------------- ①住所→座標(単発, UI用)
    def geocode(self, address):
        """住所→(lat,lon,source)。UIの「場所を探す」ボタン用。飛行はしない。"""
        lat, lon, source = geocode(address)
        return {"address": address, "lat": lat, "lon": lon, "source": source}

    # -------------------------------------------------- ③離陸前 安全チェック
    def precheck(self, side_m, alt_m, lat=None, lon=None, corners=None,
                 home_lat=None, home_lon=None):
        """離陸前の安全チェック(親切版)。実際の数値見積りを返す。

        - 電池: 巡回の総距離から推定飛行時間を出し、電池の総飛行可能時間と比べる。
        - 高さ: 法令・安全の上限内か。
        - 範囲↔フェンス整合(A): 巡回範囲の角が機体ジオフェンス内か（外部データ不要の実計算）。
        - 空港・空域近接(B): 最寄り空港との距離（同梱座標＋haversine）。※警告(飛行は止めない)
        - 飛行禁止ゾーン(D): サンプル禁止ポリゴンに掛かるか（点in多角形）。※警告
        各チェックは {ok, label, detail, level} を返す。level='block' のみが飛行可否(ok)を左右し、
        'warn' は注意喚起のみ（デモの happy-path を壊さないため）。lat/lon が無ければ位置系は省く。
        corners([[lat,lon],...] 3点以上) を渡すと、四角ではなくその多角形の実形状で評価する(UI C)。
        設計: docs/safety-design.md §8。
        """
        side_m = float(side_m)
        alt_m = float(alt_m)
        corners = _clean_corners(corners)
        # 「安全に戻れる目安」= min(目視200m, 電池の予備で帰れる距離) × 0.9（既定180m）
        # 固定値135mをやめ、理由のある値から導出（geo_safety.safe_return_limit_m）
        fence_lim = geo_safety.safe_return_limit_m(ENDURANCE_S, CRUISE_SPEED_MPS)

        if corners:
            # 多角形: 重心を中心、最遠頂点距離を範囲、周長＋往復を経路長とする
            clat = sum(c[0] for c in corners) / len(corners)
            clon = sum(c[1] for c in corners) / len(corners)
            lat, lon = clat, clon
            corner_m = max(geo_safety.haversine_m(clat, clon, c[0], c[1]) for c in corners)
            perim = 0.0
            for i in range(len(corners)):
                a, b = corners[i], corners[(i + 1) % len(corners)]
                perim += geo_safety.haversine_m(a[0], a[1], b[0], b[1])
            horizontal = perim + 2 * corner_m
            zone_points = [[clat, clon]] + corners
        else:
            # 四角: 中心→角(対角) + 四辺 + 閉じ + 帰還(対角)
            half = side_m / 2.0
            diag = half * math.sqrt(2)
            horizontal = diag + 4 * side_m + side_m + diag
            corner_m = geo_safety.corner_distance_m(side_m)
            zone_points = None
            if lat is not None and lon is not None:
                zone_points = [[float(lat), float(lon)]] + \
                    geo_safety.square_corners(float(lat), float(lon), side_m)

        est_time = horizontal / CRUISE_SPEED_MPS + alt_m / CLIMB_SPEED_MPS * 2
        batt_ok = est_time < ENDURANCE_S * 0.6   # 往復＋余裕(60%)

        # 離発着地点(ホーム)がエリアから離れている場合、フェンスはホーム中心なので
        # 「ホーム→最遠点」の距離で評価する（保守的に corner_m にホーム↔中心距離を加算）。
        if home_lat is not None and home_lon is not None and lat is not None and lon is not None:
            corner_m += geo_safety.haversine_m(float(home_lat), float(home_lon), lat, lon)
        fence_ok = corner_m <= fence_lim

        checks = {
            "battery": {
                "ok": batt_ok, "level": "block",
                "label": "電池は足ります（帰りのぶんも含めて）" if batt_ok
                         else "電池が足りません（帰りのぶんが不足）",
                "detail": "みまわり一周＋帰りで約%d分 ／ 電池のもち約%d分（使用%d%%・余裕を確保）" % (
                    round(est_time / 60), round(ENDURANCE_S / 60),
                    round(est_time / ENDURANCE_S * 100)),
            },
            "altitude": {
                "ok": 10.0 <= alt_m <= 60.0, "level": "block",
                "label": "高さは安全な範囲です" if 10.0 <= alt_m <= 60.0
                         else "高さが安全な範囲を外れています",
                "detail": "高度 %dm（安全範囲 10〜60m）" % round(alt_m),
            },
            "fence": {
                "ok": fence_ok, "level": "block",
                "label": "みまわり範囲は無理なく戻れる広さです" if fence_ok
                         else "みまわり範囲は無理なく戻れる広さではありません",
                "detail": ("いちばん遠い地点まで %dm ≦ 目安 %dm（目視%dmと電池の余力の小さい方×0.9で計算）"
                           % (round(corner_m), round(fence_lim), round(geo_safety.VLOS_WARN_M)))
                    if fence_ok else
                    ("いちばん遠い地点が %dm ＞ 目安 %dm（目視%dmと電池の余力から計算）。"
                     "ドローンが途中で自動的に引き返してしまう恐れ → 範囲を狭めてください"
                     % (round(corner_m), round(fence_lim), round(geo_safety.VLOS_WARN_M))),
            },
        }

        # 目視距離（操縦位置=離発着地点から機体が見える距離か）※フェンスより緩い"気づき"用
        vlos_ok = corner_m <= geo_safety.VLOS_WARN_M
        checks["vlos"] = {
            "ok": vlos_ok, "level": "warn",
            "label": "目視で機体を確認できる距離です" if vlos_ok
                     else "目視で機体を確認しづらい距離です",
            "detail": ("いちばん遠くても %dm（目視の目安 %dm 以内）"
                       % (round(corner_m), round(geo_safety.VLOS_WARN_M)))
                if vlos_ok else
                ("いちばん遠い地点が %dm → 機体を見失いやすい距離です。近くから操縦してください"
                 % round(corner_m)),
        }
        order = ["battery", "altitude", "fence", "vlos"]

        # 位置ベースの評価（住所を確定していれば実施）
        if lat is not None and lon is not None:
            lat = float(lat); lon = float(lon)
            near = geo_safety.nearest_airport(lat, lon)
            if near:
                aname, adist = near
                airport_ok = adist >= geo_safety.AIRPORT_WARN_M
                checks["airport"] = {
                    "ok": airport_ok, "level": "warn",
                    "label": "近くに空港はありません" if airport_ok else "近くに空港があります",
                    "detail": ("最寄りの空港（%s）まで約%.1fkm" % (aname, adist / 1000.0))
                        if airport_ok else
                        ("空港（%s）まで約%.1fkm → 空港の近くは飛行に許可が必要な場合があります"
                         % (aname, adist / 1000.0)),
                }
                order.append("airport")

            # 鉄道・送電線・学校/病院・文化財/重要施設（1回のOpenStreetMap照会でまとめて取得）。
            # 取得できない時は鉄道のみ同梱サンプルへフォールバックし、未確認である旨を明示。
            pts = zone_points or [[lat, lon]]
            search_r = max(300.0, corner_m + 200.0)
            try:
                hz = geo_safety.osm_hazards_near(pts, lat, lon, search_r)
            except Exception:
                hz = None

            # 重要施設・文化財: 同梱サンプル(皇居等) + OSM実データ(城・御所・史跡・軍事施設)を統合
            hit = geo_safety.zones_hit(pts)
            if hz is not None:
                cul = hz.get("culture")
                if cul and cul[1] < geo_safety.CULTURE_WARN_M and cul[0] not in hit:
                    hit.append(cul[0])
            nofly_ok = len(hit) == 0
            checks["nofly"] = {
                "ok": nofly_ok, "level": "warn",
                "label": "重要施設・文化財の上空にかかりません" if nofly_ok
                         else "重要施設・文化財の上空にかかります",
                "detail": "皇居・城・史跡などの上空にはかかりません（OpenStreetMap＋デモ用データ）"
                    if nofly_ok else
                    ("「%s」の上空・すぐ近くです → 飛ばすには許可・配慮が必要です（DIPS申請等）"
                     % "、".join(hit)),
            }
            order.append("nofly")

            if hz is not None:
                for key, warn_m, ok_label, ng_label, kind in (
                        ("railway", geo_safety.RAIL_WARN_M,
                         "電車の上・近くを飛びません", "電車の上・近くを飛んでしまいます", "線路"),
                        ("power",   geo_safety.POWER_WARN_M,
                         "送電線から離れています", "送電線に近すぎます", "送電線"),
                        ("crowd",   geo_safety.CROWD_WARN_M,
                         "学校・病院などの上を飛びません", "学校・病院などのすぐ近くを飛んでしまいます", "施設")):
                    item = hz.get(key)
                    if item is None:
                        checks[key] = {"ok": True, "level": "warn", "label": ok_label,
                                       "detail": "周囲%dmに%sはありません（OpenStreetMap）"
                                                 % (round(search_r), kind)}
                    else:
                        name, d = item
                        ok = d >= warn_m
                        checks[key] = {"ok": ok, "level": "warn",
                                       "label": ok_label if ok else ng_label,
                                       "detail": ("いちばん近い%s「%s」まで約%dm"
                                                  % (kind, name, round(d))) if ok else
                                                 ("%s「%s」まで約%dm → 真上や近くは危険です。ルートを離してください"
                                                  % (kind, name, round(d)))}
                    order.append(key)
            else:
                rail = geo_safety.railway_near(pts)   # オフライン等 → サンプル
                if rail:
                    rname, rdist = rail
                    rail_ok = rdist >= geo_safety.RAIL_WARN_M
                    checks["railway"] = {
                        "ok": rail_ok, "level": "warn",
                        "label": "電車の上・近くを飛びません" if rail_ok
                                 else "電車の上・近くを飛んでしまいます",
                        "detail": "【サンプル・未確認】最寄りの線路「%s」まで約%dm" % (rname, round(rdist))
                            if rail_ok else
                            "【サンプル・未確認】線路「%s」まで約%dm → ルートを離してください" % (rname, round(rdist)),
                    }
                    order.append("railway")
                checks["hazard_offline"] = {
                    "ok": False, "level": "warn",
                    "label": "送電線・学校/病院・文化財を確認できませんでした",
                    "detail": "オフラインのため確認できませんでした。インターネット接続時に自動で確認します",
                }
                order.append("hazard_offline")

            # 時間帯（日中に帰って来られるか。日の出/日の入は目安±15分）
            dl = geo_safety.daylight_status(lat, lon, est_time)
            if dl["why"] == "before":
                dl_detail = "まだ日の出前です（日の出 %s）→ 夜間の飛行は原則できません" % dl["sunrise"]
            elif dl["why"] == "ends_after":
                dl_detail = ("帰ってくる頃に日が沈みます（日の入 %s）→ 明るいうちに終えてください"
                             % dl["sunset"])
            else:
                dl_detail = "日中の飛行です（日の入 %s まで余裕あり）" % dl["sunset"]
            checks["daylight"] = {"ok": dl["ok"], "level": "warn",
                                  "label": "明るい時間帯に飛びます" if dl["ok"]
                                           else "暗い時間帯にかかってしまいます",
                                  "detail": dl_detail}
            order.append("daylight")

            # 天気（風・雨）。取得できなければ"未確認"を明示
            try:
                w = geo_safety.weather_now(lat, lon)
                reasons = []
                if w["wind"] >= geo_safety.WIND_WARN_MS:
                    reasons.append("風が強い（%.1fm/s）" % w["wind"])
                if w["gust"] >= geo_safety.GUST_WARN_MS:
                    reasons.append("突風のおそれ（%.1fm/s）" % w["gust"])
                if w["rain"] > 0:
                    reasons.append("雨（%.1fmm）" % w["rain"])
                wx_ok = not reasons
                checks["weather"] = {
                    "ok": wx_ok, "level": "warn",
                    "label": "天気は飛行に向いています" if wx_ok
                             else "天気が飛行に向いていません",
                    "detail": ("風 %.1fm/s・突風 %.1fm/s・雨なし" % (w["wind"], w["gust"]))
                        if wx_ok else
                        ("、".join(reasons) + " → 無理せず天気の回復を待ちましょう"),
                }
            except Exception:
                checks["weather"] = {
                    "ok": False, "level": "warn",
                    "label": "天気を確認できませんでした",
                    "detail": "天気を確認できませんでした。風の強い日・雨の日は飛ばさないでください",
                }
            order.append("weather")

        # 飛行可否は block レベルのみで判定（warn は止めない）
        all_ok = all(c["ok"] for c in checks.values() if c.get("level") == "block")
        return {"ok": all_ok, "checks": checks, "order": order,
                "est_time_s": round(est_time), "est_distance_m": round(horizontal)}

    # -------------------------------------------------------- ④実行(開始)
    def start(self, address=None, lat=None, lon=None, side_m=DEFAULT_SIDE_M,
              alt_m=DEFAULT_ALT_M, connect_str=None, corners=None,
              home_lat=None, home_lon=None):
        """巡回飛行を開始する。すぐ返り、実処理は別スレッドで進む。

        connect_str を渡すと、既に起動済みの SITL に接続する(プレゼンで --map を見せたい時)。
        省略時は、離発着地点(home_lat/lon)をホームにして SITL を自動起動する。
        home 未指定なら巡回エリア中心(住所/多角形重心)をホームにする(従来どおり)。
        corners([[lat,lon],...] 3点以上) を渡すと、四角ではなくその多角形の外周を巡回する(UI C)。
        """
        if self.is_running():
            raise RuntimeError("すでに飛行中です")

        corners = _clean_corners(corners)
        self._stop_flag = False
        self._state = self._initial_state()
        self._set(side_m=float(side_m), alt_m=float(alt_m), address=address,
                  lat=lat, lon=lon, phase="geocoding", message="準備中…",
                  started_at=time.time())
        if home_lat is not None and home_lon is not None:
            self._set(home_lat=float(home_lat), home_lon=float(home_lon))

        self._thread = threading.Thread(
            target=self._run,
            args=(address, lat, lon, float(side_m), float(alt_m), connect_str, corners,
                  home_lat, home_lon),
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
    def _run(self, address, lat, lon, side_m, alt_m, connect_str, corners=None,
             home_lat=None, home_lon=None):
        half = side_m / 2.0
        try:
            # 1) 住所→座標(UIが座標を渡していなければ変換)
            if lat is None or lon is None:
                self._set(phase="geocoding", message="住所を座標に変換中…")
                lat, lon, source = geocode(address or "")
                self._set(lat=lat, lon=lon, geocode_source=source)
            else:
                self._set(geocode_source="ui")

            # 多角形ルート(UI C)なら重心を中心に、最遠頂点距離を範囲(half相当)にする
            if corners:
                clat = sum(c[0] for c in corners) / len(corners)
                clon = sum(c[1] for c in corners) / len(corners)
                lat, lon = clat, clon
                half = max(geo_safety.haversine_m(clat, clon, c[0], c[1]) for c in corners)
                self._set(lat=lat, lon=lon)

            # 離発着地点(ホーム)。未指定ならエリア中心をホームにする。
            hlat = home_lat if home_lat is not None else lat
            hlon = home_lon if home_lon is not None else lon
            self._set(home_lat=hlat, home_lon=hlon)

            # 2) SITL接続(既存に接続 or 自動起動)
            target = connect_str
            if target:
                self._set(phase="connecting", message="ArduPilotに接続中…")
            else:
                self._set(phase="launching", message="シミュレータを起動中…（約30秒）")
                self._sitl = launch_sitl(hlat, hlon)   # 離発着地点をSITLホームにする
                if not wait_for_port(SIM_HOST, SIM_PORT):
                    raise RuntimeError("シミュレータの起動に失敗しました")
                target = CONNECT_STR
                self._set(phase="connecting", message="ArduPilotに接続中…")

            self._vehicle = connect(target, wait_ready=True)
            self._set(connected=True)
            # ArduPilotのメッセージ(STATUSTEXT)を購読してログ表示（③守る層の通達もここに出る）
            self._vehicle.add_message_listener(
                "STATUSTEXT",
                lambda veh, name, m: self._add_event(m.severity, m.text))
            self._add_event(6, "接続しました。ArduPilotの状態監視を開始")

            center = LocationGlobal(lat, lon, alt_m)
            self._fly(self._vehicle, center, half, alt_m, corners=corners)

            if not self._stop_flag:
                self._set(phase="done", message="みまわり完了。無事に戻りました。",
                          finished_at=time.time())
                self._set(flight_log=self._build_flight_log("完了（予定どおり帰還）"))
            else:
                # ユーザーが途中で中止 → 帰還させて終了(終端フェーズにする)
                self._set(phase="done", message="中止しました。ドローンは帰還しました。",
                          finished_at=time.time())
                self._set(flight_log=self._build_flight_log("途中で中止（自動帰還）"))
        except Exception as e:
            self._set(phase="error",
                      message="うまくいきませんでした: %s" % e,
                      error="%s\n%s" % (e, traceback.format_exc()),
                      finished_at=time.time())
            self._set(flight_log=self._build_flight_log("異常終了: %s" % e))
        finally:
            self._cleanup()

    def _fly(self, vehicle, center, half, alt_m, corners=None):
        """離陸→AUTO巡回→帰還。毎秒テレメトリを状態へ反映する。
        corners が渡されればその多角形の外周を、無ければ四角(half=半辺)を巡回する。"""
        # 準備(GPS/EKF)待ち
        self._set(phase="prearm", message="機体の準備中…（GPS受信を待っています）")
        t0 = time.time()
        while not vehicle.is_armable:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            if time.time() - t0 > 120:
                raise RuntimeError("機体の準備が整いませんでした（GPSを受信できる場所か確認してください）")
            time.sleep(1)

        # ミッション生成 + アップロード（多角形 or 四角）
        self._set(message="みまわりルートを機体に送信中…")
        if corners:
            num_corners = build_polygon_mission(vehicle, corners, alt_m)
            route = [[c[0], c[1]] for c in corners]
        else:
            num_corners = build_patrol_mission(vehicle, center, half, alt_m)
            # ※ build_patrol_mission と同じ順序・同じ座標にする
            corner_locs = [
                get_location_metres(center,  half, -half),
                get_location_metres(center,  half,  half),
                get_location_metres(center, -half,  half),
                get_location_metres(center, -half, -half),
            ]
            route = [[c.lat, c.lon] for c in corner_locs]
        # 地図描画用に、実際の巡回ルートの緯度経度と地点数を状態へ入れる
        self._set(corners=route, wp_total=num_corners)
        self._add_event(6, "ミッション送信: 離陸 + 巡回%d地点 + 帰還" % num_corners)

        # ③守る層 Step1 を実飛行に統合: 安全パラメータをこの計画に合わせて自動投入。
        # フェンスは離発着地点(ホーム)中心なので、半径=ホーム→最遠地点+余裕30m で自動フィット。
        # 高度上限も巡回高度+15m に自動フィット（設計 safety-design.md §5「統合」）。
        self._set(message="安全設定（フェンス等）を機体に送信中…")
        try:
            st = self.snapshot()
            hlat = st.get("home_lat")
            hlon = st.get("home_lon")
            if hlat is None or hlon is None:
                hlat, hlon = center.lat, center.lon
            fence_r = max(geo_safety.haversine_m(hlat, hlon, c[0], c[1]) for c in route) + 30.0
            fence_alt = alt_m + 15.0
            rtl_alt_m = max(alt_m, 15.0)   # 帰還は巡回高度以上で（低空で戻って衝突しない）
            safety_params = {
                "FENCE_ACTION": 1,                  # 超えたら自動RTL
                "FENCE_TYPE": 3,                    # 円 + 最大高度
                "FENCE_RADIUS": float(round(fence_r)),
                "FENCE_ALT_MAX": float(round(fence_alt)),
                "FENCE_ENABLE": 1,                  # 最後に有効化
                "RTL_ALT_M": float(round(rtl_alt_m)),   # m（旧RTL_ALT[cm]は現行でRTL_ALT_M[m]に改名）
                "FS_GCS_TIMEOUT": 10,               # 通信断とみなすまでの秒数(1Hzハートビートに余裕)
                "FS_GCS_ENABLE": 1,                 # 通信断→自動RTL
                "BATT_FS_LOW_ACT": 2,               # 電池低下→自動RTL
            }
            for k, v in safety_params.items():
                # 既に同じ値なら設定しない（dronekit は同値設定だと確認応答を検知できず
                # 30秒タイムアウトするため。値が違う時だけ書き込む）
                try:
                    cur = vehicle.parameters.get(k)
                except Exception:
                    cur = None
                if cur is not None and abs(float(cur) - float(v)) < 1e-3:
                    continue
                vehicle.parameters[k] = v
            self._add_event(6, "安全設定を投入: フェンス半径%dm・上限高度%dm・帰還高度%dm・通信断/電池FS有効"
                            % (round(fence_r), round(fence_alt), round(rtl_alt_m)))
        except Exception as e:
            self._add_event(4, "安全設定の投入に失敗（既定設定のまま飛行）: %s" % e)

        # アーム→離陸
        self._set(phase="takeoff", message="離陸中…")
        self._add_event(6, "GUIDEDでアーム→離陸(%dm)を指示" % round(alt_m))
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            time.sleep(1)
        vehicle.simple_takeoff(alt_m)
        t0 = time.time()
        while True:
            if self._stop_flag:
                return
            self._push_telemetry(vehicle, center, half)
            cur = vehicle.location.global_relative_frame.alt or 0.0
            if cur >= alt_m * 0.95:
                break
            if time.time() - t0 > 60:   # 無限ハング防止（60秒で上がらなければ異常）
                raise RuntimeError("離陸できませんでした（高度が上がらない）")
            time.sleep(1)

        # AUTOで巡回
        self._set(phase="patrol", message="みまわり中…")
        self._add_event(6, "AUTOモードへ切替: ミッション(巡回)を自動実行")
        vehicle.commands.next = 0
        vehicle.mode = VehicleMode("AUTO")
        last_seq = 1 + num_corners + 1   # takeoff(1) + 角(num) + 閉じ(1)
        # 完了検出を堅牢化: 最終到達 / 最終WP付近で進捗停止 / 安全タイムアウト の三段構え
        # （ミッション完走後に commands.next が進まず"⑤で固まる"のを防ぐ）
        patrol_deadline = time.time() + 90 + num_corners * 45
        prev_next, stall = -1, 0
        while True:
            if self._stop_flag:
                break
            self._push_telemetry(vehicle, center, half)
            # 安全機構(フェンス/電池FS等)が介入すると、機体側が勝手にRTL/LANDへ切り替わる。
            # それを検知して、人に分かる言葉で画面に伝える（③守る層の見せ場）。
            mode_name = vehicle.mode.name if vehicle.mode else ""
            if mode_name in ("RTL", "LAND"):
                self._add_event(4, "安全機構が作動: モードが%sに自動切替" % mode_name)
                self._set(message="安全機構が作動しました。自動でもどっています…")
                break
            nxt = vehicle.commands.next
            if nxt >= last_seq:
                self._add_event(6, "巡回ミッション完了（全%d地点）" % num_corners)
                break
            # 最終付近(全WP到達済)で進捗が10秒止まったら完了とみなす
            if nxt == prev_next:
                stall += 1
            else:
                stall, prev_next = 0, nxt
            if nxt >= (1 + num_corners) and stall >= 10:
                self._add_event(6, "巡回ミッション完了（最終地点で待機を検出）")
                break
            if time.time() > patrol_deadline:
                self._add_event(4, "巡回がタイムアウト。安全のため帰還します")
                break
            time.sleep(1)

        # 帰還(RTL)。安全機構が既にLANDを選んでいたら邪魔しない
        self._set(phase="rtl", message="もどっています…（自動帰還）")
        cur_mode = vehicle.mode.name if vehicle.mode else ""
        if cur_mode != "LAND":
            if cur_mode != "RTL":
                self._add_event(6, "RTLモードへ切替: 離発着地点へ自動帰還")
            vehicle.mode = VehicleMode("RTL")
        t0 = time.time()
        while time.time() - t0 < 120:
            self._push_telemetry(vehicle, center, half)
            if not vehicle.armed:
                break
            cur = vehicle.location.global_relative_frame.alt
            if cur is not None and cur < 1.5:   # 着地(高度が有効かつ低い)。Noneでは誤終了しない
                break
            time.sleep(1)
        self._add_event(6, "着地・ディスアームを確認（みまわり終了）")

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

            wp_total = self._state.get("wp_total", NUM_CORNERS)
            wp_next = vehicle.commands.next
            wp_index = max(0, min(wp_total, wp_next - 1))
            wp_dist = distance_to_current_waypoint(vehicle)

            # ArduPilot 内部の状態（可視化用）。取得失敗は None のまま。
            gps = vehicle.gps_0
            try:
                sys_status = vehicle.system_status.state if vehicle.system_status else None
            except Exception:
                sys_status = None

            self._set(
                mode=vehicle.mode.name if vehicle.mode else None,
                armed=bool(vehicle.armed),
                alt=round(alt, 1),
                max_alt=round(max(alt, self._state.get("max_alt") or 0.0), 1),
                battery=level,
                attitude_ok=attitude_ok,
                in_range=in_range,
                battery_ok=battery_ok,
                wp_index=wp_index,
                wp_distance=round(wp_dist) if wp_dist is not None else None,
                veh_lat=veh_lat,
                veh_lon=veh_lon,
                ekf_ok=bool(vehicle.ekf_ok) if vehicle.ekf_ok is not None else None,
                gps_fix=gps.fix_type if gps else None,
                gps_sats=gps.satellites_visible if gps else None,
                armable=bool(vehicle.is_armable),
                sys_status=sys_status,
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
