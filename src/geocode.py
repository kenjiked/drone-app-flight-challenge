#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""住所/地名 → 緯度経度（ジオコーディング）。

無料の Nominatim (OpenStreetMap) を使う。APIキー不要。
- スクショから座標を推定するのは不正確で危険なので採用しない(design-decisions D3)。
  住所→座標は枯れた技術で正確なので、こちらを使う(D2)。
- ネットワーク不通/失敗時は、内蔵テーブル(FALLBACK)にフォールバックしてデモを止めない。

使い方:
  python3 src/geocode.py "東京駅"
  python3 src/geocode.py "名古屋城"
"""
from __future__ import print_function

import json
import sys
import urllib.parse
import urllib.request

NOMINATIM = "https://nominatim.openstreetmap.org/search"
# Nominatim はデフォルトの python UA をブロックするので、必ず名乗る
USER_AGENT = "drone-app-flight-challenge/0.1 (team demo)"

# オンライン失敗時のフォールバック（デモ用の代表地点）
FALLBACK = {
    "東京駅": (35.681236, 139.767125),
    "tokyo station": (35.681236, 139.767125),
    "名古屋駅": (35.170694, 136.881637),
    "大阪駅": (34.702485, 135.495951),
}


def geocode(address, timeout=6):
    """address を (lat, lon, source) に変換する。source は 'online' か 'fallback'。
    どちらでも解決できなければ RuntimeError。"""
    # 1) オンライン(Nominatim)
    try:
        q = urllib.parse.urlencode({"q": address, "format": "json", "limit": 1})
        req = urllib.request.Request(NOMINATIM + "?" + q,
                                     headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            data = json.load(r)
        if data:
            return float(data[0]["lat"]), float(data[0]["lon"]), "online"
        print("  (オンライン: 該当なし) フォールバックを試します", file=sys.stderr)
    except Exception as e:
        print("  (オンライン失敗: %s) フォールバックを試します" % type(e).__name__,
              file=sys.stderr)

    # 2) フォールバック(内蔵テーブル)
    key = address.strip().lower()
    for k, v in FALLBACK.items():
        if k.lower() == key:
            return v[0], v[1], "fallback"

    raise RuntimeError("住所を座標化できませんでした: %r" % address)


def main():
    if len(sys.argv) < 2:
        print('usage: python3 src/geocode.py "住所または地名"')
        sys.exit(2)
    address = " ".join(sys.argv[1:])
    lat, lon, source = geocode(address)
    print("%s -> lat=%.6f lon=%.6f (%s)" % (address, lat, lon, source))
    # スクリプト連携用に、機械可読な行も最後に出す (lat,lon)
    print("%.6f,%.6f" % (lat, lon))


if __name__ == "__main__":
    main()
