#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""みまわり自作ドローン — Webバックエンド(標準ライブラリのみ)。

ブラウザUI(ui/app.html)とArduPilot(SITL)を繋ぐHTTPサーバ。
Flask等の追加依存は使わず、Python標準の http.server で動かす(オフライン・簡単)。

提供するもの:
  GET  /                → ui/app.html を配信
  GET  /api/status      → 現在の飛行状態(実テレメトリ)を JSON で返す
  POST /api/geocode     → {address} → {lat,lon,source}          (①場所を探す)
  POST /api/precheck    → {side,alt} → 安全チェック結果          (③飛ばす前チェック)
  POST /api/start       → {address,lat,lon,side,alt,connect?} 巡回開始 (④実行)
  POST /api/stop        → 帰還させて停止

使い方:
  python3 src/server.py                 # http://127.0.0.1:8000 を開く
  DRONE_CONNECT=tcp:127.0.0.1:5760 python3 src/server.py   # 起動済みSITLに接続

デモの流れ:
  ブラウザで住所入力 → 裏でSITLを起動/接続 → AUTOで巡回 → 画面に実データを表示。
"""
from __future__ import print_function

import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# src/ 内の自作モジュールを import できるようにする
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_service import manager, DEFAULT_SIDE_M, DEFAULT_ALT_M  # noqa: E402

HOST = os.environ.get("DRONE_HOST", "127.0.0.1")
PORT = int(os.environ.get("DRONE_PORT", "8000"))
# 指定があれば、起動済みSITLに接続する(自動起動しない)。プレゼンで --map を見せたい時に。
FORCE_CONNECT = os.environ.get("DRONE_CONNECT")

UI_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ui")
UI_FILE = os.path.join(UI_DIR, "app.html")


class Handler(BaseHTTPRequestHandler):
    # ログを静かに(必要なら消す)
    def log_message(self, fmt, *args):
        sys.stderr.write("  %s - %s\n" % (self.address_string(), fmt % args))

    # ---------------------------------------------------------------- helpers
    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path, content_type):
        try:
            with open(path, "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404, "not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except ValueError:
            return {}

    # -------------------------------------------------------------------- GET
    def do_GET(self):
        if self.path in ("/", "/index.html", "/app.html"):
            self._send_file(UI_FILE, "text/html; charset=utf-8")
        elif self.path == "/mockup.html":
            self._send_file(os.path.join(UI_DIR, "mockup.html"), "text/html; charset=utf-8")
        elif self.path == "/api/status":
            self._send_json(manager.snapshot())
        elif self.path == "/api/config":
            self._send_json({"side_m": DEFAULT_SIDE_M, "alt_m": DEFAULT_ALT_M,
                             "auto_launch": FORCE_CONNECT is None})
        else:
            self.send_error(404, "not found")

    # ------------------------------------------------------------------- POST
    def do_POST(self):
        try:
            if self.path == "/api/geocode":
                data = self._read_json()
                addr = (data.get("address") or "").strip()
                if not addr:
                    return self._send_json({"error": "住所が空です"}, 400)
                self._send_json(manager.geocode(addr))

            elif self.path == "/api/precheck":
                data = self._read_json()
                side = float(data.get("side", DEFAULT_SIDE_M))
                alt = float(data.get("alt", DEFAULT_ALT_M))
                self._send_json(manager.precheck(side, alt))

            elif self.path == "/api/start":
                data = self._read_json()
                self._send_json(manager.start(
                    address=(data.get("address") or None),
                    lat=data.get("lat"),
                    lon=data.get("lon"),
                    side_m=float(data.get("side", DEFAULT_SIDE_M)),
                    alt_m=float(data.get("alt", DEFAULT_ALT_M)),
                    connect_str=FORCE_CONNECT,
                ))

            elif self.path == "/api/stop":
                self._send_json(manager.stop())

            else:
                self.send_error(404, "not found")
        except Exception as e:
            self._send_json({"error": str(e)}, 400)


def main():
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    mode = ("起動済みSITLに接続 (%s)" % FORCE_CONNECT) if FORCE_CONNECT else "SITLを自動起動"
    print("=" * 56)
    print(" みまわり自作ドローン — Webサーバ起動")
    print("  URL   : http://%s:%d" % (HOST, PORT))
    print("  動作  : %s" % mode)
    print("  停止  : Ctrl+C")
    print("=" * 56)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n停止します…")
    finally:
        try:
            manager.stop()
        except Exception:
            pass
        server.server_close()


if __name__ == "__main__":
    main()
