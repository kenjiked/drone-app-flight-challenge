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

## メモ

- ログ(*.tlog)は .gitignore 済み。必要なら要点だけここに転記する。
