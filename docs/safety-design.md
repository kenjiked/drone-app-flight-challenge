# 安全設計（③守る層）— フライトモードまで落とし込んだ設計

> 「素人が作った飛行計画を、機体側が見守る安全機構」（design-decisions.md）の具体化。
> ArduCopter の実ソース（`~/GitHub/ardupilot`）を読んで、**どの異常が・どの設定で・どのフライトモードを発動するか**まで確定した記録。
> file:line は ArduPilot リポジトリ内の位置（設計の裏付け・学習用）。

## 0. 前提の整理：なぜ「独自モードで監視」は噛み合わないか（積み残しの決着）

design-decisions.md の未解決事項だった論点:
> 巡回は標準 **AUTO** で飛ばす。しかし AUTO 飛行中は別モードにできないため「独自モードで監視」はそのまま噛み合わない。

**実ソースで確認した事実**（`ArduCopter/mode.cpp`）:
- フライトモードは**常にどれか1つだけ**。監視のために別モードへ移ることはできない。
- ただし ArduPilot の安全機構は「監視して、危なくなったら**フライトモードを切り替える**」構造になっている。
  失敗理由（`ModeReason`）付きで `Copter::set_mode()` を呼ぶ（`ArduCopter/mode.cpp:313`）。
  例：`ModeReason::FENCE_BREACHED`(=10)、`BATTERY_FAILSAFE`(=4)、`EKF_FAILSAFE`(=6)、`SCRIPTING`(=32)（`libraries/AP_Vehicle/ModeReason.h:19`）。

→ **決定（D21）**: ③安全機構は「独自モード」ではなく、
**(a) 常時監視フェイルセーフ＋ジオフェンス（＝設定）** を土台に、
**(c) 自作条件は Lua スクリプトで監視し、必要時に `vehicle:set_mode()` でモード切替** を足す、ハイブリッドにする。
AUTO で巡回 → 異常時は各安全機構が **RTL / LAND / BRAKE** を発動、という形が ArduPilot 本来の設計と一致する。

→ **決定（D22）**: 実装言語は **まず Lua**（再コンパイル不要・反復が速い・安全機構の全体像を先に掴む）。
深掘りしたい箇所が定まったら C++（`AP_Arming_Copter` / 各モード）へ降りる。design-decisions の
「まず動かすなら Lua、深く学ぶなら C++」に沿う。※SITL は再ビルド不要で Lua を試せるのも後押し。

---

## 1. 安全の3層モデル

| 層 | いつ | 何をする | 実現手段 |
|---|---|---|---|
| **L1 離陸前** | アーム前 | 無理な計画・危険な状態なら**飛ばさない** | ARMING_CHECK（標準）＋ Lua aux-auth（自作） |
| **L2 飛行中の常時監視** | 飛行中ずっと | 異常を検知して**安全なフライトモードへ自動遷移** | フェイルセーフ群＋ジオフェンス（設定）＋ Lua 監視 |
| **L3 収束・通知** | 異常後 | 確実に帰す/降ろす・人に知らせる | RTL/LAND/SmartRTL の挙動＋ GCS 通知 |

UI との対応:
- UI ③「飛ばす前チェック」（電池/範囲/高さ） = **L1**（今は Python の張りぼて判定 → Lua aux-auth で本物に）
- UI ⑤「姿勢/範囲/電池」チップ = **L2**（今は表示だけ → 本物のフェイルセーフ発動と連動。チップが「注意」= 実際にモードが RTL に変わる）

---

## 2. L2 中核：異常 → 設定 → 発動フライトモード 対応表

これが「フライトモードまでの具体化」の本体。すべて実ソースの実パラメータ名・実モード名・実 ModeReason。

| # | 異常 | 検知 | 設定パラメータ（推奨値） | 発動フライトモード | ModeReason | 根拠(file:line) |
|---|---|---|---|---|---|---|
| 1 | **範囲逸脱**（外周から出る／高度超過） | AC_Fence | `FENCE_ENABLE=1` / `FENCE_TYPE=3`(円+最大高度) / `FENCE_RADIUS`=巡回半径+余裕 / `FENCE_ALT_MAX`=高度+余裕 / `FENCE_ACTION=1` | **RTL**（ただし breach>100m は強制 **LAND**） | `FENCE_BREACHED` | `AC_Fence.cpp:67`, `ArduCopter/fence.cpp:72-107` |
| 2 | **電池 低下** | BattMonitor | `BATT_LOW_VOLT`/`BATT_LOW_MAH` + `BATT_FS_LOW_ACT=2`(RTL) / `BATT_LOW_TIMER=10` | **RTL** | `BATTERY_FAILSAFE` | `AP_BattMonitor_Params.cpp:117,153`, `events.cpp:99` |
| 3 | **電池 危険** | BattMonitor | `BATT_CRT_VOLT`/`BATT_CRT_MAH` + `BATT_FS_CRT_ACT=1`(LAND) | **LAND**（その場で降下＝最優先で降ろす） | `BATTERY_FAILSAFE` | `AP_BattMonitor_Params.cpp:133,165` |
| 4 | **通信(GCS)途絶** | GCS FS | `FS_GCS_ENABLE=1` / `FS_GCS_TIMEOUT=5` | **RTL** | `GCS_FAILSAFE` | `Parameters.cpp:96`, `events.cpp:163` |
| 5 | **送信機(RC)途絶** | Radio FS | `FS_THR_ENABLE=1`(always RTL) / `FS_THR_VALUE=975` | **RTL** | `RADIO_FAILSAFE` | `Parameters.cpp:124`, `events.cpp:13` |
| 6 | **位置推定異常(EKF/GPS)** | EKF check | `FS_EKF_ACTION=1`(Land) / `FS_EKF_THRESH=0.8` | **LAND**（位置不要モードで安全降下） | `EKF_FAILSAFE` | `Parameters.cpp:267`, `ekf_check.cpp:166` |
| 7 | **自作条件**（姿勢崩れ・想定外の速度・巡回中心から一定以上 等） | Lua 監視 | `SCR_ENABLE=1` + スクリプト | **RTL**（`vehicle:set_mode(6)`）または LAND(9) | `SCRIPTING` | `bindings.desc:351`, `docs.lua:2868` |

### なぜそのモードになるのか（モードの性質）
実ソースで各モードの `requires_position()` を確認済み（`ArduCopter/mode.h`）:
- **RTL(6)** … ホームへ自動帰還→着陸。**位置が要る**（`mode.h:1512`）。範囲逸脱・電池低下・通信断など「まだ位置は取れている」異常の既定。
- **LAND(9)** … その場で自動降下。**位置が無くても着陸できる**（`mode.h:1303`）。だから **GPS/EKF 喪失**（#6）や**電池危険**（#3）の“最後の砦”はこれ。RTL に頼れない場面で確実に降ろす。
- **SMART_RTL(21)** … 通った経路を逆走して帰る。障害物の多い畑向き。`BATT_FS_LOW_ACT=3` 等で選択可。**位置＋経路バッファが要る**（`mode.h:1646`）。
- **BRAKE(17)** … 慣性/GPSで即停止。**位置が要る**（`mode.h:878`）。「まず止めて考える」用途（`*_ACT` の Brake or Land 系）。
- **AUTO(3)** … 巡回ミッション本体。位置が要る（`mode_auto.cpp:178`）。

### 重要な設計事実（実ソースで確認）
- **AUTO のミッションが終わっても自動で RTL しない**。既定は上空なら **LOITER**、失敗時 **LAND**（`ModeReason::MISSION_END`, `mode_auto.cpp:846-861`）。
  → だから巡回後の帰還は**明示的に RTL を出す**必要がある（現状の `patrol_spine.py` は `vehicle.mode="RTL"` を出しており正しい）。
  ミッション内で帰還着陸させたいなら `DO_LAND_START` を置き **AUTO_RTL(27)** を使う（`mode.cpp:345-351`）。
- **位置が無いと位置要求モードへは入れない**。`requires_position() && !position_ok()` で拒否（`mode.cpp:391`）。
  だから GPS 喪失時の砦が **LAND**（位置不要）なのは理にかなう。
- **フェイルセーフの優先度**：`Copter.h:642` の `_failsafe_priorities[]` が `TERMINATE > LAND > RTL > SmartRTL...` の順。
  例：電池が既に LAND を出していれば、後から来た通信断が RTL に**格下げしない**（`events.cpp:53-56`）。
- **ジオフェンスは 100m 超過で問答無用の LAND**（`AC_FENCE_GIVE_UP_DISTANCE`, `fence.cpp:72`）。遠くまで暴走したら帰還を諦めて降ろす、という思想。

---

## 3. L1 離陸前チェック（pre-arm）の具体化

### 標準（設定のみ）
`ARMING_CHECK` ビットマスク（`AP_Arming.h:26`）で GPS/INS/COMPASS/BATTERY 等を強制。`ARMING_CHECK=1` で全部。

### 自作チェック（Lua aux-auth）— UI ③ の“本物”
`ARMING_CHECK` の **AUX_AUTH ビット(=1<<17)** を使うと、Lua から離陸可否をブロックできる（`AP_Arming.cpp:1568`）。
参考実装が本家にある: `libraries/AP_Scripting/applets/arming-checks.lua`。

流れ:
1. 起動時に1回 `local id = arming:get_aux_auth_id()`
2. 未アームの間ループし、
   - 全チェックOK → `arming:set_aux_auth_passed(id)`
   - どれかNG → `arming:set_aux_auth_failed(id, "みまわり: 電池不足のため離陸できません")`（GCSに表示＝離陸拒否）

自作チェック内容（UI ③ と一致させる）:
- 電池残量 `battery:capacity_remaining_pct(0)` が閾値以上か
- 巡回範囲がフェンス内か（`FENCE_RADIUS`/`FENCE_ALT_MAX` と計画を突き合わせ）
- ホーム取得済み `ahrs:get_home()` / 位置健全 `ahrs:get_location()`

---

## 4. L3 収束・通知

- **RTL の挙動**：ホーム上空へ戻り、`RTL_ALT` 高度で帰還→降下→`LAND` 相当で着陸→ディスアーム。
- **通知**：Lua から `gcs:send_text(MAV_SEVERITY, "…")` で理由を人に伝える。UI ⑤ のチップ/メッセージへ反映。
- **FS_OPTIONS**：着陸中は継続する等の微調整（`Copter.h:611`）。デモでは既定でよい。

---

## 5. 実装ロードマップ（walking skeleton of safety）

段階的に。各段でデモに“見せ場”が増える。

### Step 1 — 設定だけ（コード0行・即デモ可）
巡回開始前に、我々のスタックからパラメータを流し込む（統合＝チームのテーマ D10/D11）。
`patrol_spine.py`/`flight_service.py` で dronekit の `vehicle.parameters[...]=...`、または `.parm` を読み込む。

推奨初期値（デモ用、巡回サイズ・高度に応じて計算）:
```
FENCE_ENABLE     1
FENCE_TYPE       3          # 円(2) + 最大高度(1)
FENCE_RADIUS     <巡回半径 + 30>    # 例: 一辺160m→半径~113m → 150
FENCE_ALT_MAX    <巡回高度 + 15>    # 例: 25m → 40
FENCE_ACTION     1          # RTL or Land
BATT_LOW_VOLT    10.5       # 3S目安（機体に合わせる）
BATT_FS_LOW_ACT  2          # RTL
BATT_CRT_VOLT    10.0
BATT_FS_CRT_ACT  1          # LAND
FS_GCS_ENABLE    1          # 通信断→RTL
FS_EKF_ACTION    1          # 位置喪失→LAND
```
**デモの見せ場**：巡回中にわざと `FENCE_RADIUS` を小さくする／機体を外周外へ誘導 → **自動で RTL** する様子を、UI ⑤ の「範囲 注意」＋モード表示が `AUTO→RTL` に変わるのと同時に見せる。

### Step 2 — Lua（自作 pre-arm＋飛行中監視）
`safety/patrol_safety.lua`（新規予定）を SITL の `scripts/` に置き `SCR_ENABLE=1`。
- pre-arm：`arming-checks.lua` を土台に、電池%・範囲・ホームで aux-auth。
- 飛行中：`battery:capacity_remaining_pct(0)` / `ahrs:get_roll_rad()`,`get_pitch_rad()` / `ahrs:get_home():get_distance(ahrs:get_location())` を監視、閾値超で `vehicle:set_mode(6)`（RTL）。
  参考テンプレ：`applets/copter-deadreckon-home.lua`（GPS喪失で Guided_NoGPS→RTL する実例）。
- 使うバインディング（実在確認済み, `docs.lua`）:
  `arming:get_aux_auth_id / set_aux_auth_passed / set_aux_auth_failed`,
  `ahrs:get_roll_rad / get_pitch_rad / get_location / get_home`, `Location:get_distance`,
  `battery:capacity_remaining_pct / voltage`, `vehicle:get_mode / set_mode`, `gcs:send_text`。

### Step 3 — C++（深掘り・任意）
学びたい箇所が定まったら：`AP_Arming_Copter.cpp` に自作 pre-arm を1つ足す（`check_failed(...)`）、
または特定モードの `init()`/`run()` を読んで挙動を理解・微修正。design-decisions D12 の「既存に1個足して深く理解」。

---

## 6. デモ台本への一言（発表用）

> 「素人の計画を、機体が見守る」を具体化しました。範囲を出れば**ジオフェンスが RTL**、電池が危なければ
> **バッテリーフェイルセーフが RTL→最後は LAND**、GPS を失えば **EKF フェイルセーフが LAND**。
> どれも“監視して、危なくなったらフライトモードを自動で切り替える”という ArduPilot 本来の安全設計に乗せています。
> まずは設定で成立させ、次に自作の離陸前チェックと飛行中監視を Lua で足し、最終的に C++ で深掘りします。

---

## 7. 参照した実ソース（学習ポインタ）

- モード定義・遷移: `ArduCopter/mode.h`, `ArduCopter/mode.cpp`, `ArduCopter/mode_auto.cpp`
- 失敗理由: `libraries/AP_Vehicle/ModeReason.h`
- フェイルセーフ: `ArduCopter/events.cpp`, `ArduCopter/ekf_check.cpp`, `ArduCopter/Parameters.cpp`, `ArduCopter/Copter.h`
- バッテリ: `libraries/AP_BattMonitor/AP_BattMonitor_Params.cpp`
- フェンス: `libraries/AC_Fence/AC_Fence.cpp/.h`, `ArduCopter/fence.cpp`
- アーミング: `libraries/AP_Arming/AP_Arming.cpp/.h`, `ArduCopter/AP_Arming_Copter.cpp`
- Lua: `libraries/AP_Scripting/applets/arming-checks.lua`, `applets/copter-deadreckon-home.lua`,
  `libraries/AP_Scripting/docs/docs.lua`, `generator/description/bindings.desc`

---

## 8. 計画側ルート安全性評価：未実装アイデア（ロードマップ）

> ③守る層のうち **「地図・法令に基づくルートの事前評価」** は計画側（Web バックエンド `src/`）の役割。
> 現状の `flight_service.py::precheck()` は **電池・範囲・高さ** の幾何/エネルギー見積り（親切版）のみ実装済み。
> 以下は**まだ実装していないアイデア**（バックエンド未実装）。UI ③ には**モックの評価UI**として
> "評価中→判定(✓/⚠)" のアニメで表示し、実装後の体験を先取りしている（`ui/app.html` の `mockEvaluate`／
> 「モック・デモ用・実データではありません」と明示）。うち **建物高さは巡回高度に連動**、
> **範囲↔フェンス整合は実計算**（角 = 一辺/2×√2 vs 135m）で"効いてる感"を出す。飛行判定には影響しない。
> 役割分担: 機体側(Lua/FW)は「機体が知っている状態」、計画側は「地図・法令の情報」。

| アイデア | 何を評価するか | 想定データ源 | 状態 |
|---|---|---|---|
| **巡回範囲とジオフェンスの整合(A)** | 計画範囲の角が機体フェンス内か（実 `FENCE_RADIUS×0.9` と突合） | 自スタックのパラメータ | ✅ 実装（`geo_safety`／level=block） |
| **空港・制限空域の近接(B)** | 最寄り空港との距離 | 同梱の主要空港座標＋haversine | ✅ 実装（5km圏内で warn） |
| **注意ゾーン(D)** | ルートが重要施設(皇居等)・文化財(城/寺社)に掛かるか | 同梱サンプル多角形＋点in多角形 | ✅ 実装（warn／データは例示。公式DID・文化財GISに差し替え余地） |
| **鉄道の上・近接** | 電車の上/60m以内を飛ばないか | **実データ: OpenStreetMap(Overpass)** 地上路線のみ(地下鉄・トンネル除外)。オフライン時は同梱サンプル(未確認と明示) | ✅ 実装（warn） |
| **送電線の近接** | 送電線60m以内(接触・干渉) | 実データ: OSM `power=line/minor_line`（鉄道と同一クエリで一括取得） | ✅ 実装（warn） |
| **学校・病院など人が集まる施設** | 施設100m以内/上空(第三者上空回避) | 実データ: OSM `amenity=school/hospital/kindergarten/大学` | ✅ 実装（warn） |
| **時間帯（夜間飛行）** | いま飛んで日中(日の出〜日の入)に帰れるか。推定飛行時間を加味 | オフライン計算（太陽赤緯・目安±15分） | ✅ 実装（warn） |
| **天気（風・突風・雨）** | 風5m/s・突風10m/s・降雨で注意 | Open-Meteo API（無料・キー不要）。取得不可時は「未確認」と明示 | ✅ 実装（warn） |
| **目視距離(VLOS)** | 操縦位置(ホーム)から200m超は見失いやすい | 実計算（フェンスより緩い"気づき"レベル） | ✅ 実装（warn） |
| **機体登録・リモートID** | 100g以上の登録義務 | 判定せず注意書きをUI③に常時表示 | ✅ 表示 |
| **建物・障害物の高さ** | ルート上の建物高さ vs 巡回高度 | 建物高さ／3D都市モデル(PLATEAU等) | ⬜ モックのまま（データ入手が重い） |
| **地形の起伏（対地高度）** | 標高データで対地高度を保てるか | DEM／ArduPilot Terrain | ⬜ モックのまま |

> **実装メモ（2026-07-02）**: A/B/D は `src/geo_safety.py`（外部API不要・オフライン）で実チェック化。
> `precheck()` が各 check に `level`（block=飛行可否を左右／warn=注意のみ）を付与し、位置(lat/lon)や
> 多角形(corners)にも対応。建物高さ・地形AGLはデータが重いため引き続きモック（UI `mockEvaluate`）。

### 多角形ルート（地図をなぞる, D8）— 実装（2026-07-02）
UI で地図をタップして頂点を打つ→その多角形の外周を巡回（`patrol_spine.build_polygon_mission`）。
SITL 実飛行で 5 頂点ポリゴンの AUTO 巡回→RTL を確認（sitl-test-log.md）。precheck も多角形の実形状で評価。

### 既知の不整合 → 解消（A で対応済み, 2026-07-02）
- 旧: `precheck()` の範囲上限 260m（固定）が機体 `FENCE_RADIUS=150`（Lua ×0.9=135m）と食い違い、
  「OK」で通ったルートが離陸直後に自作安全機構で即 RTL される恐れがあった。
- 現: 範囲チェックを**フェンス整合の実計算（角 vs 135m, level=block）**へ置換。一辺≒191m 超（角>135m）は
  pre-flight でブロックし、フェンス発動を未然に防ぐ。§3 の「範囲を実フェンス値と突き合わせる」意図に合致。

> これらは「地図/法令ベースのルート安全性評価」という③の本丸であり、チームのテーマ（非エンジニア向けの
> わかりやすさ＋統合, D10/D11）の主戦場。外部データAPI連携が要るため、まず**警告表示（飛行は止めない）**から
> 段階導入するのが現実的。
