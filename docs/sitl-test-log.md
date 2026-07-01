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
| 2026-07-02 | **UI↔ArduPilot連結**: ブラウザ→HTTP→dronekit→SITL (`src/server.py` + `flight_service.py` + `ui/app.html`) | server.py がAPI経由でSITL自動起動 | `tcp:127.0.0.1:5760` | **OK（完走・実テレメトリ確認）** | `/api/start`(名古屋城,一辺100m,高度20m)→launching→connecting→prearm→takeoff(0→20m)→AUTO巡回(wp 1→4)→RTL(20→1.5m)→done。モード遷移 STABILIZE→GUIDED→AUTO→RTL、電池 100→83%、姿勢/範囲/電池フラグを実データで取得。急旋回で姿勢フラグが一瞬 false=実測の証拠。SITLはdone後に0プロセスへ自動片付け |

## 発見・修正（背骨テスト 2026-07-01）

- **バグ**: GPS/EKF準備前に現在地を読み、巡回中心が (0,0)＝ヌル島になっていた。
  → 修正: `wait_until_ready()` を追加し、armable になってから現在地を取得（`src/patrol_spine.py`）。
- dronekit の `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` 警告は旧ミッションプロトコル由来。動作に影響なし（将来 `_INT` 版へ寄せる余地）。
- 次の一歩: 前段に「住所→座標(ジオコーディング)」、後段に「離陸前チェック/飛行中の実データ監視」を接続する。

## UI↔ArduPilot 連結（2026-07-02）

- モックUI(`ui/mockup.html`)の6ステップの張りぼてを、実データに置き換えて `ui/app.html` を作成:
  ①住所→実ジオコーディング / ③飛行前チェック→実数値見積り / ⑤飛行中→実テレメトリ(毎秒ポーリング)。
- バックエンド `src/server.py`(標準ライブラリ `http.server`, Flask不要) が
  `src/flight_service.py`(飛行を別スレッドで実行し状態を共有) 経由で dronekit/SITL を駆動。
- 既存の `geocode.py` / `patrol_spine.py` / `plan_and_fly.py` を再利用（車輪の再発明を避ける）。
- ⑤の「姿勢OK・範囲内」は固定文字をやめ、機体の実姿勢(±30°)・中心からの距離・電池残量で判定（親切版=Python側監視）。
  ArduPilotフライトコード側の深い拡張(③の芯, design D7/D18)は次段階。
- 起動: `python3 src/server.py` → `http://127.0.0.1:8000`。`DRONE_CONNECT=...` で起動済みSITLに接続も可。

## メモ

- ログ(*.tlog)は .gitignore 済み。必要なら要点だけここに転記する。
