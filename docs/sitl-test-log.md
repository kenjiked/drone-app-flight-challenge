# SITL 動作確認ログ

ArduPilot SITL で課題を検証した記録を残す。

## 環境

- OS: Windows + WSL2 (Ubuntu 22.04)
- ArduPilot: ~/GitHub/ardupilot (board=sitl, ArduCopter ビルド済み)
- 接続先(例): `udp:127.0.0.1:14550`

## 記録表

| 日付 | 課題 | SITL起動コマンド | 接続先 | 結果 | 備考 |
|---|---|---|---|---|---|
| YYYY-MM-DD | 例) arm & takeoff | `sim_vehicle.py -v ArduCopter --console --map` | `udp:127.0.0.1:14550` | OK / NG | |
| 2026-07-01 | 背骨: 座標→四角巡回→AUTO飛行→RTL (`src/patrol_spine.py`) | `sim_vehicle.py -v ArduCopter --no-mavproxy -w` | `tcp:127.0.0.1:5760` | **OK（完走）** | 中心=SITLホーム(-35.36,149.15)。arm→離陸15m→4地点巡回→帰還を確認。walking skeleton 一本通った |
| 2026-07-01 | 頭: 住所→座標→その場所で巡回 (`src/geocode.py` + `patrol_spine.py`) | `sim_vehicle.py -v ArduCopter --no-mavproxy --custom-location=<lat,lon,0,0> -w` | `tcp:127.0.0.1:5760` | **OK（完走）** | 「名古屋城」→(35.181605,136.905495)をSITLホームにして起動→巡回中心が名古屋城座標で一致→完走。ジオコーディング統合を確認 |
| 2026-07-01 | 1コマンド体験: 住所→SITL起動→巡回→自動片付け (`src/plan_and_fly.py`) | スクリプトが内部で自動起動 | `tcp:127.0.0.1:5760` | **OK（完走・自動片付け確認）** | `python3 src/plan_and_fly.py "名古屋城"` の1発で全工程。離陸は数秒スロットル立ち上がり後に正常上昇→4地点巡回→RTL→SITL停止まで自動。①→②が1コマンドに統合 |

## 発見・修正（背骨テスト 2026-07-01）

- **バグ**: GPS/EKF準備前に現在地を読み、巡回中心が (0,0)＝ヌル島になっていた。
  → 修正: `wait_until_ready()` を追加し、armable になってから現在地を取得（`src/patrol_spine.py`）。
- dronekit の `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` 警告は旧ミッションプロトコル由来。動作に影響なし（将来 `_INT` 版へ寄せる余地）。
- 次の一歩: 前段に「住所→座標(ジオコーディング)」、後段に「離陸前チェック/飛行中の実データ監視」を接続する。

## メモ

- ログ(*.tlog)は .gitignore 済み。必要なら要点だけここに転記する。
