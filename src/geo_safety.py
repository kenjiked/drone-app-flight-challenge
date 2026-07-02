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

# 鉄道近接の警告しきい値[m]（真上〜近接は墜落時に重大。線路からの水平距離）
RAIL_WARN_M = 60.0

# 送電線近接の警告しきい値[m]（接触・電磁干渉）
POWER_WARN_M = 60.0

# 人が集まる施設（学校・病院など）近接の警告しきい値[m]（第三者上空を避ける）
CROWD_WARN_M = 100.0

# 目視で機体を確認しづらくなる距離の目安[m]（操縦位置=ホームから）
VLOS_WARN_M = 200.0

# 天候の警告しきい値（風[m/s]・突風[m/s]・降水[mm]）
WIND_WARN_MS = 5.0
GUST_WARN_MS = 10.0

# amenity 種別の日本語名（名称タグが無い施設向け）
AMENITY_JA = {"school": "学校", "kindergarten": "幼稚園・保育園",
              "hospital": "病院", "college": "大学・専門学校", "university": "大学"}

# --- 注意ゾーン（サンプル・デモ用）。[名称, 種別, [[lat,lon],...]] の多角形 ---
# 種別: "重要施設"(皇居・国会等) / "文化財"(城・寺社等)。飛行が制限・要配慮な代表例。
# ※データは "例示" 止まり。実運用は国土地理院DID・文化財GIS等の公式データに差し替える。
CAUTION_ZONES = [
    ("皇居", "重要施設", [
        [35.6905, 139.7440], [35.6905, 139.7590],
        [35.6790, 139.7590], [35.6790, 139.7440],
    ]),
    ("国会議事堂", "重要施設", [
        [35.6790, 139.7420], [35.6790, 139.7490],
        [35.6730, 139.7490], [35.6730, 139.7420],
    ]),
    # 名古屋城（デモ既定地点＝文化財）。ジオコード結果の揺れを吸収するため広めに取る。
    ("名古屋城", "文化財", [
        [35.1880, 136.8970], [35.1880, 136.9080],
        [35.1780, 136.9080], [35.1780, 136.8970],
    ]),
    ("二条城", "文化財", [
        [35.0155, 135.7470], [35.0155, 135.7520],
        [35.0125, 135.7520], [35.0125, 135.7470],
    ]),
]
# 後方互換（旧名で参照するコード向け）
NOFLY_ZONES = [(n, poly) for (n, _kind, poly) in CAUTION_ZONES]

# --- 鉄道路線（サンプル・近似の折れ線）。[名称, [[lat,lon],...]] ---
# 「電車の上」を飛ばない/近づかないための近接判定に使う。※データは例示。
RAILWAYS = [
    ("JR山手線/東京〜有楽町(例)", [
        [35.6812, 139.7671], [35.6750, 139.7636], [35.6699, 139.7630],
    ]),
    ("JR中央本線/名古屋〜金山(例)", [
        [35.1706, 136.8816], [35.1580, 136.8880], [35.1430, 136.8990],
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
    """points([[lat,lon],...]) のいずれかが掛かる注意ゾーンの "種別:名称" 一覧。"""
    hit = []
    for name, kind, poly in CAUTION_ZONES:
        if any(_point_in_polygon(p[0], p[1], poly) for p in points):
            label = "%s:%s" % (kind, name)
            if label not in hit:
                hit.append(label)
    return hit


def _segment_distance_m(plat, plon, alat, alon, blat, blon):
    """点P(plat,plon) と線分AB の最短水平距離[m]。P周りの局所平面へ投影して計算。"""
    latref = math.radians(plat)

    def xy(lat, lon):
        x = math.radians(lon - plon) * math.cos(latref) * 6371000.0
        y = math.radians(lat - plat) * 6371000.0
        return x, y

    ax, ay = xy(alat, alon)
    bx, by = xy(blat, blon)   # P は原点(0,0)
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 == 0.0:
        return math.hypot(ax, ay)
    t = -(ax * dx + ay * dy) / seg2      # 原点(P)の AB 上への射影パラメータ
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(cx, cy)


def railway_near(points):
    """points([[lat,lon],...]) が最も近い鉄道までの (名称, 距離m)。同梱サンプルのみ。線路が無ければ None。"""
    best = None
    for name, line in RAILWAYS:
        for i in range(len(line) - 1):
            a, b = line[i], line[i + 1]
            for p in points:
                d = _segment_distance_m(p[0], p[1], a[0], a[1], b[0], b[1])
                if best is None or d < best[1]:
                    best = (name, d)
    return best


def _min_dist_to_line(points, line):
    """points と折れ線 line の最短距離[m]。"""
    best = None
    for i in range(len(line) - 1):
        a, b = line[i], line[i + 1]
        for p in points:
            d = _segment_distance_m(p[0], p[1], a[0], a[1], b[0], b[1])
            if best is None or d < best:
                best = d
    return best


def railway_near_online(points, center_lat, center_lon, radius_m, timeout=8):
    """OpenStreetMap(Overpass)で実際の鉄道を取得し、points からの最短 (名称, 距離m) を返す。
    半径内に鉄道が無ければ None(=安全)。取得失敗時は例外を送出（呼び側でサンプルにフォールバック）。"""
    import json as _json
    import urllib.request as _req
    import urllib.parse as _parse

    r = max(200, int(radius_m))
    # 地上路線のみ（地下鉄subwayは上空リスクが低いので除外）。tunnel=yes も後段で除外。
    query = (
        "[out:json][timeout:%d];"
        '(way["railway"~"^(rail|light_rail|tram|monorail|narrow_gauge)$"]'
        "(around:%d,%f,%f););out geom tags;" % (timeout, r, center_lat, center_lon)
    )
    data = _parse.urlencode({"data": query}).encode()
    req = _req.Request("https://overpass-api.de/api/interpreter", data=data,
                       headers={"User-Agent": "drone-app-flight-challenge/1.0"})
    with _req.urlopen(req, timeout=timeout) as resp:
        obj = _json.loads(resp.read().decode("utf-8"))

    best = None
    for el in obj.get("elements", []):
        geom = el.get("geometry") or []
        if len(geom) < 2:
            continue
        tags0 = el.get("tags", {})
        if tags0.get("tunnel") in ("yes", "building_passage") or tags0.get("location") == "underground":
            continue   # 地下・トンネルは上空リスクが低いので除外
        line = [[g["lat"], g["lon"]] for g in geom]
        d = _min_dist_to_line(points, line)
        if d is None:
            continue
        tags = el.get("tags", {})
        name = tags.get("name") or tags.get("railway") or "鉄道"
        if best is None or d < best[1]:
            best = (name, d)
    return best   # None なら半径内に鉄道なし


def osm_hazards_near(points, center_lat, center_lon, radius_m, timeout=8):
    """1回の Overpass 照会で 鉄道・送電線・人が集まる施設(学校/病院等) を取得し、
    カテゴリごとの最短 (名称, 距離m) を dict で返す（無いカテゴリはキー無し）。
    地下鉄・トンネルは上空リスクが低いので除外。取得失敗時は例外（呼び側でフォールバック）。"""
    import json as _json
    import urllib.request as _req
    import urllib.parse as _parse

    r = max(200, int(radius_m))
    around = "(around:%d,%f,%f)" % (r, center_lat, center_lon)
    amen = '"amenity"~"^(school|kindergarten|hospital|college|university)$"'
    query = (
        "[out:json][timeout:%d];("
        'way["railway"~"^(rail|light_rail|tram|monorail|narrow_gauge)$"]%s;'
        'way["power"~"^(line|minor_line)$"]%s;'
        "node[%s]%s;way[%s]%s;"
        ");out geom;" % (timeout, around, around, amen, around, amen, around)
    )
    data = _parse.urlencode({"data": query}).encode()
    req = _req.Request("https://overpass-api.de/api/interpreter", data=data,
                       headers={"User-Agent": "drone-app-flight-challenge/1.0"})
    with _req.urlopen(req, timeout=timeout) as resp:
        obj = _json.loads(resp.read().decode("utf-8"))

    res = {}
    for el in obj.get("elements", []):
        tags = el.get("tags", {})
        if "railway" in tags:
            if tags.get("tunnel") in ("yes", "building_passage") or tags.get("location") == "underground":
                continue
            cat, name = "railway", (tags.get("name") or "鉄道")
        elif "power" in tags:
            cat, name = "power", (tags.get("name") or "送電線")
        elif "amenity" in tags:
            cat = "crowd"
            name = tags.get("name") or AMENITY_JA.get(tags["amenity"], "施設")
        else:
            continue

        if el.get("type") == "node":
            d = min(haversine_m(p[0], p[1], el["lat"], el["lon"]) for p in points)
        else:
            geom = el.get("geometry") or []
            if len(geom) < 2:
                continue
            d = _min_dist_to_line(points, [[g["lat"], g["lon"]] for g in geom])
        if d is None:
            continue
        cur = res.get(cat)
        if cur is None or d < cur[1]:
            res[cat] = (name, d)
    return res


def daylight_status(lat, lon, duration_s):
    """いま飛んで日中(日の出〜日の入)に収まるかの目安（誤差±15分程度）。
    返り値: {ok, sunrise:'HH:MM', sunset:'HH:MM', why: None|'before'|'ends_after'}"""
    import datetime as _dt

    now = _dt.datetime.now().astimezone()
    n = now.timetuple().tm_yday
    decl = -23.44 * math.cos(math.radians(360.0 / 365.0 * (n + 10)))
    cosw = -math.tan(math.radians(lat)) * math.tan(math.radians(decl))
    cosw = max(-1.0, min(1.0, cosw))
    w = math.degrees(math.acos(cosw))          # 半日弧[deg]
    noon_utc_h = 12.0 - lon / 15.0             # 太陽南中(UTC時)
    base = _dt.datetime(now.year, now.month, now.day, tzinfo=_dt.timezone.utc)
    sr = (base + _dt.timedelta(hours=noon_utc_h - w / 15.0)).astimezone(now.tzinfo)
    ss = (base + _dt.timedelta(hours=noon_utc_h + w / 15.0)).astimezone(now.tzinfo)
    end = now + _dt.timedelta(seconds=float(duration_s) + 300)   # 帰還+片付けの余裕5分

    why = None
    if now < sr:
        why = "before"
    elif end > ss:
        why = "ends_after"
    return {"ok": why is None, "sunrise": sr.strftime("%H:%M"),
            "sunset": ss.strftime("%H:%M"), "why": why}


def weather_now(lat, lon, timeout=6):
    """現在の天気（Open-Meteo・無料/キー不要）。{wind, gust, rain} [m/s, m/s, mm]。
    取得失敗時は例外（呼び側で"確認できず"扱い）。"""
    import json as _json
    import urllib.request as _req

    url = ("https://api.open-meteo.com/v1/forecast?latitude=%f&longitude=%f"
           "&current=wind_speed_10m,wind_gusts_10m,precipitation&wind_speed_unit=ms"
           % (lat, lon))
    req = _req.Request(url, headers={"User-Agent": "drone-app-flight-challenge/1.0"})
    with _req.urlopen(req, timeout=timeout) as resp:
        cur = _json.loads(resp.read().decode("utf-8")).get("current", {})
    return {"wind": float(cur.get("wind_speed_10m") or 0.0),
            "gust": float(cur.get("wind_gusts_10m") or 0.0),
            "rain": float(cur.get("precipitation") or 0.0)}
