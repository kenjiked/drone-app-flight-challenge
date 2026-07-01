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

## ディレクトリ構成

```
src/        課題本体のコード
examples/   SITL 接続サンプル
docs/       ドキュメント（SITL 動作確認ログ含む）
```

## ドキュメント

環境・提出:
- [docs/setup-progress.md](docs/setup-progress.md) — 環境構築・提出準備の進捗記録
- [docs/sitl-test-log.md](docs/sitl-test-log.md) — SITL 動作確認ログ

認識合わせデモ（コース3卒業課題）:
- [docs/design-decisions.md](docs/design-decisions.md) — 設計判断の記録（何を・なぜ）※まずここ
- [docs/concept-demo.md](docs/concept-demo.md) — デモの目的・背景・スコープ
- [docs/requirements-draft.md](docs/requirements-draft.md) — 必須/追加/やらないこと
- [docs/team-mtg-agenda.md](docs/team-mtg-agenda.md) — 次回MTGアジェンダ
- [docs/role-assignment-draft.md](docs/role-assignment-draft.md) — 役割分担・ボトルネック
- [docs/demo-script.md](docs/demo-script.md) — デモ進行台本
- [docs/presentation-outline.md](docs/presentation-outline.md) — 発表資料アウトライン
- [docs/speaker-notes.md](docs/speaker-notes.md) — 発表・説明原稿

## SITL 動作確認記録

テスト結果は [docs/sitl-test-log.md](docs/sitl-test-log.md) に記録する。
