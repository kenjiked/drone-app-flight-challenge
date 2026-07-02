--[[
みまわり安全スクリプト (patrol_safety.lua) — ③守る層 Step2(Lua)

役割:
  L1 離陸前(pre-arm)   : 電池・位置が不十分なら「離陸を止める」(aux-auth で arming をブロック)
  L2 飛行中の常時監視  : 姿勢・電池・巡回範囲を監視し、危険なら自動で RTL(帰還)へフライトモードを切替

設計根拠は docs/safety-design.md。ここは「親切版(自作条件)」の監視で、
ArduPilot 標準のフェンス/バッテリ/GCS/EKF フェイルセーフ(設定=Step1)の上に重ねる。
標準FSが拾わない/より早く優しく止めたい条件をカバーする。

使い方(SITL):
  1) 機体パラメータ  SCR_ENABLE = 1
  2) 本ファイルを SITL の scripts/ ディレクトリに置く
  3) 再起動(reboot)。起動後 "PatrolSafety x.y loaded" が GCS に出れば有効
  （範囲監視は FENCE_RADIUS を読むので、Step1の safety/patrol_safety.parm を先に入れると効く）

使用バインディングはすべて実在確認済み(libraries/AP_Scripting/docs/docs.lua)。
--]]

local SCRIPT_NAME    = "PatrolSafety"
local SCRIPT_VERSION = "0.1"

-- GCS メッセージ severity(重大度)
local MAV_SEVERITY = { EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7 }

-- ArduCopter フライトモード番号(ArduCopter/mode.h)
local MODE_AUTO, MODE_GUIDED, MODE_RTL, MODE_LAND = 3, 4, 6, 9

-- ===== 調整パラメータ(v0.1 は定数。将来スクリプトパラメータ化してもよい) =====
local ARM_MIN_BATT_PCT = 40     -- これ未満なら離陸させない(%)
local FLY_LOW_BATT_PCT = 30     -- 飛行中これ未満で RTL(%)
local ATT_LIMIT_DEG    = 45     -- ロール/ピッチがこれを超えたら RTL(度)
local DIST_FRAC        = 0.9    -- ホームから FENCE_RADIUS*この割合 を超えたら RTL(標準フェンスより一歩手前で優しく)
local LOOP_MS          = 1000   -- 監視周期(ms)
local STARTUP_MS       = 15000  -- 起動直後は AP 初期化待ち(スパム防止)

-- 実行時に一度だけ確保する
local auth_id = arming:get_aux_auth_id()
local FENCE_RADIUS = Parameter("FENCE_RADIUS")  -- Copter は常に存在
local intervened = false        -- 今回の飛行で既に自動介入したか(RTLを毎秒撃たない)

-- ---- 危険時: RTL へ切替 ----
-- 注: GCS へ送る文字列は短い ASCII に統一する。
--   理由(1) MAVLink STATUSTEXT は 50 バイト上限 → 日本語(UTF-8マルチバイト)は途中切れする。
--   理由(2) GCS/ツールによっては非ASCIIを表示できず文字化けする。
-- 非エンジニア向けの日本語表示は Web UI 側の役割(設計の役割分担)。ここは機体側の技術メッセージ。
local function trigger_rtl(reason)
  gcs:send_text(MAV_SEVERITY.WARNING,
    string.format("%s: %s -> RTL", SCRIPT_NAME, reason))
  if vehicle:set_mode(MODE_RTL) then
    intervened = true
  else
    gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": RTL switch FAILED")
  end
end

-- ---- L1: 離陸前チェック(未アーム中に毎周期報告) ----
local function pre_arm_report()
  local ok = true
  local reasons = {}

  -- 電池残量(取得できた時だけ判定。容量未設定なら標準チェックに委ねる)
  local pct = battery:capacity_remaining_pct(0)
  if pct and pct < ARM_MIN_BATT_PCT then
    ok = false
    reasons[#reasons+1] = string.format("batt %d%%<%d%%", pct, ARM_MIN_BATT_PCT)
  end

  -- 位置推定が確定しているか(ヌル島=中心(0,0)で飛ぶ事故を防ぐ)
  local loc = ahrs:get_location()
  if not loc then
    ok = false
    reasons[#reasons+1] = "no position (GPS/EKF)"
  end

  if ok then
    arming:set_aux_auth_passed(auth_id)
  else
    -- "Arm: " は ArduPilot が前置。ASCII短文で 50 バイト制限内に収める(先頭 "PS:" が本スクリプト印)
    arming:set_aux_auth_failed(auth_id, "PS: " .. table.concat(reasons, " / "))
  end
end

-- ---- L2: 飛行中の監視 ----
local function in_flight_monitor()
  -- 我々の自動巡回(AUTO/GUIDED)の時だけ介入する。
  -- 操縦者や標準FSが既に RTL/LAND 等へ切替えていたら邪魔しない。
  local mode = vehicle:get_mode()
  if mode ~= MODE_AUTO and mode ~= MODE_GUIDED then return end
  if intervened then return end

  -- 姿勢(ロール/ピッチ)の乱れ
  local roll  = math.abs(math.deg(ahrs:get_roll_rad()))
  local pitch = math.abs(math.deg(ahrs:get_pitch_rad()))
  if roll > ATT_LIMIT_DEG or pitch > ATT_LIMIT_DEG then
    trigger_rtl(string.format("attitude %.0f/%.0f deg", roll, pitch))
    return
  end

  -- 電池(親切版: 標準の電圧FSより早めに帰す)
  local pct = battery:capacity_remaining_pct(0)
  if pct and pct < FLY_LOW_BATT_PCT then
    trigger_rtl(string.format("battery low %d%%", pct))
    return
  end

  -- 範囲: ホーム(=巡回中心)からの距離が フェンス半径*DIST_FRAC を超えたら一歩手前で帰す
  local fr = FENCE_RADIUS:get()
  if fr and fr > 0 then
    local home = ahrs:get_home()
    local loc  = ahrs:get_location()
    if home and loc then
      local d = home:get_distance(loc)   -- 水平距離(m)
      if d > fr * DIST_FRAC then
        trigger_rtl(string.format("out of range %.0fm>%.0fm", d, fr * DIST_FRAC))
        return
      end
    end
  end
end

-- ---- メインループ ----
local function update()
  if not arming:is_armed() then
    intervened = false          -- 着地・ディスアームで次の飛行に備えてリセット
    pre_arm_report()
  else
    in_flight_monitor()
  end
  return update, LOOP_MS
end

-- ---- 起動 ----
if not auth_id then
  gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ": no aux_auth id (ARMING_CHECK)")
  return
end
gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s loaded", SCRIPT_NAME, SCRIPT_VERSION))
return update, STARTUP_MS
