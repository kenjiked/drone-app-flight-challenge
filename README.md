# drone-app-flight-challenge

ドローンエンジニア養成塾の課題提出・ソース共有用リポジトリ。
ArduPilot SITL + MAVProxy + pymavlink / DroneKit-Python を使って動作確認する。

## 開発環境

- Windows + WSL2 (Ubuntu 22.04)
- Python 3.10
- ArduPilot SITL (ArduCopter)
- MAVProxy / pymavlink 2.4.49 / dronekit 2.9.2

## セットアップ

```bash
git clone https://github.com/<YOUR_USERNAME>/drone-app-flight-challenge.git
cd drone-app-flight-challenge

# （任意）仮想環境
python3 -m venv venv && source venv/bin/activate

pip install -r requirements.txt
```

## 動作確認方法

1. ArduPilot SITL を起動する（別ターミナル）:

   ```bash
   cd ~/GitHub/ardupilot
   Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
   ```

2. 本リポジトリのスクリプトを実行して SITL に接続する:

   ```bash
   python3 src/<your_script>.py --connect udp:127.0.0.1:14550
   ```

   - デフォルトの MAVLink 出力先（例）: `udp:127.0.0.1:14550`
   - 接続確認だけなら `mavproxy.py --master=udp:127.0.0.1:14550`

## Webアプリ（住所→巡回をブラウザから）★チーム発表デモ

ブラウザのUIから住所を入れるだけで、裏で ArduPilot(SITL) を起動・接続し、
四角の外周巡回を AUTO で飛ばして、実テレメトリ（モード・高度・巡回進捗・姿勢・範囲・電池）を
画面に表示する。追加ライブラリ不要（Python標準ライブラリのみ）。

```bash
# サーバを起動（SITLは住所の座標をホームに自動起動する）
python3 src/server.py
# → ブラウザで http://127.0.0.1:8000 を開く

# （任意）先に SITL を --map 付きで起動しておき、それに接続する場合:
#   別ターミナル: cd ~/GitHub/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter --map
DRONE_CONNECT=tcp:127.0.0.1:5760 python3 src/server.py
```

構成: ブラウザ(`ui/app.html`) → `src/server.py`(HTTP) → `src/flight_service.py` →
dronekit/MAVLink → ArduPilot(SITL)。設計は `ui/mockup.html`（張りぼて版）を実データに置き換えたもの。

## CLI（1コマンド体験）

UIを使わず、コマンド1発で住所→巡回まで通すこともできる:

```bash
python3 src/plan_and_fly.py "名古屋城"
```

## ディレクトリ構成

```
src/        課題本体のコード（server.py / flight_service.py / patrol_spine.py / geocode.py / plan_and_fly.py）
ui/         ブラウザUI（app.html=LIVE版, mockup.html=設計モック）
examples/   SITL 接続サンプル
docs/       ドキュメント（SITL 動作確認ログ含む）
```

## ドキュメント

環境・提出:
- [docs/setup-progress.md](docs/setup-progress.md) — 環境構築・提出準備の進捗記録
- [docs/sitl-test-log.md](docs/sitl-test-log.md) — SITL 動作確認ログ

認識合わせデモ（コース3卒業課題）:
- [docs/design-decisions.md](docs/design-decisions.md) — 設計判断の記録（何を・なぜ）※まずここ
- [docs/safety-design.md](docs/safety-design.md) — ③安全機構の設計（異常→発動フライトモードの対応表・実装ロードマップ）
- [docs/concept-demo.md](docs/concept-demo.md) — デモの目的・背景・スコープ
- [docs/requirements-draft.md](docs/requirements-draft.md) — 必須/追加/やらないこと
- [docs/team-process.md](docs/team-process.md) — チームの進行方法（開発の回し方・ルール・ドキュメント地図）
- 役割分担（担当者名入り）・MTG議事録 — **非公開のチーム内リポで管理**（D23。docs内は案内スタブ）
- [docs/demo-script.md](docs/demo-script.md) — デモ進行台本
- [docs/presentation-outline.md](docs/presentation-outline.md) — 発表資料アウトライン
- [docs/speaker-notes.md](docs/speaker-notes.md) — 発表・説明原稿

## SITL 動作確認記録

テスト結果は [docs/sitl-test-log.md](docs/sitl-test-log.md) に記録する。
