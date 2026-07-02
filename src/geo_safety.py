#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地図・法令ベースのルート安全性評価（オフライン・依存ライブラリ無し）。

precheck から使う「本物の」判定ロジックを集約する:
  - フェンス整合 (A): 巡回範囲の角が機体ジオフェンス内か（外部データ不要）
  - 空港・空域近接 (B): 同梱した主要空港座標との最寄り距離（haversine）
  - 飛行禁止ゾーン (D): 同梱したサンプル禁止ポリゴンに掛かるか（点in多角形）

データは「デモに十分な最小の同梱データ」。建物高さ・地形AGLは重いデータが要るため対象外。
設計: docs/safety-design.md §8。
"""
from __future__ import print_function

import math

# 機体側 safety/patrol_safety.parm と対応させる（Lua は FENCE_RADIUS*0.9 で一歩手前）
FENCE_RADIUS_M = 150.0
FENCE_FRAC = 0.9

# 空港近接の警告しきい値[m]（デモ用の proxy。実運用は空域区分で厳密化）
AIRPORT_WARN_M = 5000.0

# --- 主要空港（名称, 緯度, 経度）。デモに十分な範囲の最小セット ---
AIRPORTS = [
    ("東京/羽田", 35.5494, 139.7798),
    ("成田", 35.7720, 140.3929),
    ("中部/セントレア", 34.8584, 136.8054),
    ("県営名古屋(小牧)", 35.2549, 136.9243),
    ("大阪/伊丹", 34.7855, 135.4382),
    ("関西", 34.4273, 135.2440),
    ("神戸", 34.6328, 135.2238),
    ("新千歳", 42.7752, 141.6923),
    ("福岡", 33.5859, 130.4506),
    ("那覇", 26.1958, 127.6459),
    ("仙台", 38.1397, 140.9169),
    ("広島", 34.4361, 132.9196),
    ("鹿児島", 31.8034, 130.7194),
    ("高松", 34.2142, 134.0156),
    ("熊本", 32.8373, 130.8551),
]

# --- 飛行禁止ゾーン（サンプル・デモ用）。[名称, [[lat,lon],...]] の多角形 ---
# ※データは "例示" 止まり。実運用は国土地理院DID等の公式ポリゴンに差し替える。
NOFLY_ZONES = [
    ("皇居周辺(例)", [
        [35.6905, 139.7440], [35.6905, 139.7590],
        [35.6790, 139.7590], [35.6790, 139.7440],
    ]),
    ("国会議事堂周辺(例)", [
        [35.6790, 139.7420], [35.6790, 139.7490],
        [35.6730, 139.7490], [35.6730, 139.7420],
    ]),
]


def haversine_m(lat1, lon1, lat2, lon2):
    """2点間の地表距離[m]。"""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def square_corners(lat, lon, side_m):
    """中心(lat,lon)・一辺 side_m の四角の4角 [[lat,lon],...]。
    patrol_spine の build_patrol_mission と同じ順序・同式（半辺=side/2）。"""
    h = side_m / 2.0
    R = 6378137.0

    def offset(dN, dE):
        dlat = dN / R
        dlon = dE / (R * math.cos(math.pi * lat / 180.0))
        return [lat + dlat * 180 / math.pi, lon + dlon * 180 / math.pi]

    return [offset(h, -h), offset(h, h), offset(-h, h), offset(-h, -h)]


def corner_distance_m(side_m):
    """四角の角＝中心からの最遠距離[m]（= 半辺*√2）。"""
    return (side_m / 2.0) * math.sqrt(2.0)


def fence_limit_m():
    """Lua が使う一歩手前しきい値[m]（FENCE_RADIUS*0.9）。"""
    return FENCE_RADIUS_M * FENCE_FRAC


def nearest_airport(lat, lon):
    """(名称, 距離m) の最寄り空港。データが無ければ None。"""
    best = None
    for name, alat, alon in AIRPORTS:
        d = haversine_m(lat, lon, alat, alon)
        if best is None or d < best[1]:
            best = (name, d)
    return best


def _point_in_polygon(lat, lon, poly):
    """レイキャスティングで点が多角形内か。poly=[[lat,lon],...]。"""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        yi, xi = poly[i][0], poly[i][1]
        yj, xj = poly[j][0], poly[j][1]
        if ((yi > lat) != (yj > lat)) and \
           (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def zones_hit(points):
    """points([[lat,lon],...]) のいずれかが掛かる禁止ゾーン名の一覧。"""
    hit = []
    for name, poly in NOFLY_ZONES:
        if any(_point_in_polygon(p[0], p[1], poly) for p in points):
            if name not in hit:
                hit.append(name)
    return hit
