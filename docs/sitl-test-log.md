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
| 2026-07-02 | **③守る層 機体側安全機構の検証**: Step1 param + Step2 Lua (`safety/patrol_safety.parm` + `patrol_safety.lua`) | `sim_vehicle.py -v ArduCopter --no-mavproxy -w`、Lua を `ardupilot/scripts/` に配置、param投入後 reboot | `tcp:127.0.0.1:5760` | **OK（Lua ロード / L1 / L2 すべて確認）** | 下記「機体側安全機構の検証」参照 |

## ルート安全性評価の実装＆多角形ルート（2026-07-02）

計画側のルート安全性評価を「モック→本物」に昇格し、多角形ルートを追加した（design-decisions D8）。

- **A/B/D 実チェック化**（`src/geo_safety.py` 新規・外部API不要・オフライン）:
  - A 範囲↔フェンス整合＝実計算（角 vs FENCE_RADIUS×0.9=135m）。level=block で飛行可否を左右。
  - B 空港近接＝主要空港座標＋haversine（5km圏内で warn）。D 飛行禁止ゾーン＝サンプルGeoJSON＋点in多角形（warn）。
  - `precheck(side,alt,lat,lon,corners)` が各 check に level(block/warn) を付けて返す。
  - 検証(単体): 名古屋城既定=全通過 / 一辺260m=fence NG(ok=false) / 皇居=nofly warn / 羽田近く=空港 warn / 位置なし=位置系省略。
- **C 多角形ルート（地図をなぞる）**: UIで頂点をタップ→ポリゴン→`build_polygon_mission` でミッション化。
  - **SITL 実飛行 OK**: ホーム周りの5頂点ポリゴンで `build_polygon_mission` が takeoff+5頂点+閉じ=7コマンドを生成、
    AUTO で seq2→7 まで全ウェイポイントを通過し一周完了→RTL。多角形の巡回が実機(SITL)で成立。
  - precheck も多角形の実形状（重心・最遠頂点・周長）で評価することを単体確認（大ポリゴン=fence/battery NG、皇居ポリゴン=nofly warn）。

## 安全設定の実飛行統合＋飛行日誌（2026-07-02）

③守る層 Step1 を UI からの実飛行に統合し、本番経路（アプリがSITLを自前起動）で完走を確認した。

- **安全パラメータ自動投入**: ミッション送信直後に計画へ自動フィットした設定を投入。
  一辺60m → `フェンス半径72m(最遠42m+30m)・上限高度30m(15+15)・帰還高度15m・通信断/電池FS有効`。
  launching→connecting→prearm→takeoff→patrol→rtl→**done** 完走、エラーゼロ。
- **発見と修正**:
  1. `FS_GCS_ENABLE=1` は倍速SITLでタイムアウト(5s)が実質短縮されGCSフェイルセーフが連発
     →`FS_GCS_TIMEOUT=10` で解消（等速の本番経路では安定）。
  2. dronekit は**既定値と同じ値**を設定すると確認応答を検知できずタイムアウト → 同値スキップで回避。
  3. 現行 ArduPilot では **`RTL_ALT`[cm] が `RTL_ALT_M`[m] に改名**（`mode_rtl.cpp` の変換表で確認）
     → 旧名のままだと無反応で設定失敗。新名+単位mへ修正。
- **飛行日誌の自動生成**: 終了時に日時・場所・離発着地点・地点数・最高高度・結果・経過を
  テキスト生成（`flight_log`）。⑥完了画面からダウンロード可。中止/エラー時も生成される。
- **安全機構の介入検知**: 巡回中に機体が自前で RTL/LAND へ切替わったら「安全機構が作動」として
  イベント＋画面メッセージに反映（フェンス/電池FSの発動がUIで見える）。

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

## 機体側安全機構の検証（③守る層 / 2026-07-02）

`safety/patrol_safety.lua`（Step2）＋ `safety/patrol_safety.parm`（Step1）を SITL で実動検証した。

**手順**: Lua を `~/GitHub/ardupilot/scripts/` に配置 → SITL 起動 → pymavlink で parm を投入 →
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` で reboot（スクリプトはブート時ロードのため）。

- **Lua ロード**: ✅ 再起動後 GCS に `PatrolSafety 0.1 loaded` (sev INFO) を確認。`SCR_ENABLE=1` は
  reboot をまたいで保持。`get_aux_auth_id` 成功＝使用バインディングはすべて実在・有効（Lua エラー無し）。
- **L1 離陸前ブロック（低バッテリ）**: ✅ `BATT_CAPACITY=20`(mAh) で残量を 0%(<40%) に落とし arm 試行 →
  `COMMAND_ACK result=4 (FAILED)`／`armed=False`。拒否理由 STATUSTEXT は `Arm: <みまわり>: <電池残量が少ない>(0% …`
  ＝自作 aux-auth のテンプレート（末尾 `(0%` は Lua の `電池残量が少ない(%d%% < %d%%)` に一致。標準チェックは
  英語 "Battery 1 low" なので別物）。→ **aux-auth による離陸ブロックが機能**。
- **L2 飛行中監視→自動RTL（範囲逸脱）**: ✅ 標準フェンスと切り分けるため `FENCE_ENABLE=0`（`FENCE_RADIUS=60` は
  Lua が `Parameter` で読む）にして GUIDED でホーム北 150m へ移動。ホームから約 59m 地点（閾値 `60×DIST_FRAC(0.9)=54m` 超）で
  自作 Lua が `巡回範囲の外に出そうです(59m > 54m) → RTLで帰還します` を送信し `vehicle:set_mode(RTL=6)` で
  **GUIDED→RTL を自動切替**。標準フェンス無効下で RTL したので、切替の主体が自作 Lua であることを確定。

**発見（要フォロー / 修正候補）**:
- ~~Lua が送る**日本語 GCS メッセージが文字化け**する~~ **解決（2026-07-02、下記「文字化け修正」参照）**:
  この pymavlink 受信経路では非ASCIIバイトが U+FFFD に置換され、加えて MAVLink **STATUSTEXT は 50 バイト上限**で
  日本語の理由文が途中切れしていた。→ 機体→GCS の安全メッセージを**短い ASCII**（数値は保持）に統一して解消。

## 文字化け修正（③守る層メッセージのASCII化 / 2026-07-02）

`patrol_safety.lua` の `gcs:send_text` / `set_aux_auth_failed` の**送信文字列を短い ASCII に統一**した
（ソースコードのコメントは日本語のまま）。理由: (1) STATUSTEXT 50バイト上限でUTF-8日本語が途中切れ、
(2) 受信ツールによっては非ASCIIが文字化け。非エンジニア向けの日本語表示は Web UI 側の役割（設計の役割分担）。

主な変更（例）:
- 範囲逸脱: `巡回範囲の外に…` → `PatrolSafety: out of range %.0fm>%.0fm -> RTL`
- 姿勢/電池: → `attitude %.0f/%.0f deg` / `battery low %d%%`
- 離陸前拒否理由: `みまわり: 電池残量が少ない(…)` → `PS: batt %d%%<%d%%` / `no position (GPS/EKF)`

**再検証（すべて OK・ASCIIで可読・50バイト以内）**:
- Lua ロード: `PatrolSafety 0.1 loaded`
- L1 離陸前ブロック: 残量0%で arm → `ACK=4 FAILED`／`armed=False`、理由 `Arm: PS: batt 0%<40%`（`isascii=True`）
- L2 範囲逸脱→RTL: FENCE無効で隔離し北へ飛行 → `PatrolSafety: out of range 60m>54m -> RTL`（`isascii=True`, 41B）で GUIDED→RTL
- 注: L1 の低残量は「先に飛行して消費を溜め→`BATT_CAPACITY` を小さく設定」で `capacity_remaining_pct` を 0% にして再現
  （放電はモーター通電＝飛行中に進むため。容量を極小(=5)にすると残量が -1=無効になり Lua はスキップする点も確認）。

**検証時の一時変更（本番設定ではない）**: L2 隔離のため `FENCE_ENABLE=0`、閾値到達を早めるため `FENCE_RADIUS=60`（parm 既定は 150/1）。
巡回サイズに応じた本番値は `safety/patrol_safety.parm` のとおり。

## メモ

- ログ(*.tlog)は .gitignore 済み。必要なら要点だけここに転記する。
