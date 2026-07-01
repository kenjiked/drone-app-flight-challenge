# Setup Progress

## Overview

- **目的**: ドローンエンジニア養成塾の課題提出・ソース共有用リポジトリの環境構築と提出準備の記録
- **使用環境**: Windows + WSL2 (Ubuntu 22.04.5 LTS) / Claude Code は Ubuntu 側で利用
- **GitHub repo URL**: https://github.com/kenjiked/drone-app-flight-challenge

## Completed

| 項目 | 状態 | メモ |
|---|---|---|
| WSL2 / Ubuntu 22.04 環境 | ✅ 完了 | Ubuntu 22.04.5 LTS、WSL2 確認済み |
| ArduPilot / SITL / MAVProxy / pymavlink / dronekit / Docker | ✅ 完了 | 診断上すべて動作可（下表参照） |
| GitHub CLI (gh) インストール | ✅ 完了 | gh 2.95.0 |
| gh auth login | ✅ 完了 | account: kenjiked |
| git config user.name / user.email | ✅ 完了 | kenjiked / noreply メール使用 |
| 提出用 repo 作成 (~/GitHub/drone-app-flight-challenge) | ✅ 完了 | ローカル + GitHub 両方 |
| 初期ファイル構成 | ✅ 完了 | README / requirements / .gitignore / src / examples / docs |
| 初回 commit | ✅ 完了 | "Initial project structure" (6c04bc1) |
| GitHub repo 作成 + 初回 push | ✅ 完了 | visibility: public、branch: master |
| SITL 起動確認 & ログ記録 | ⬜ 未完了 | 次のアクション（docs/sitl-test-log.md へ記録） |
| 提出形式の確定（URL共有 or PR） | ⚠️ 要確認 | 講師に確認が必要 |

## Environment

| 項目 | 状態 | 確認コマンド | メモ |
|---|---|---|---|
| WSL2 / Ubuntu 22.04 | ✅ 完了 | `grep PRETTY_NAME /etc/os-release` / `grep -i microsoft /proc/version` | Ubuntu 22.04.5 LTS, WSL2 |
| Git | ✅ 完了 | `git --version` | git 2.34.1 |
| GitHub CLI | ✅ 完了 | `gh --version` | gh 2.95.0 (2026-06-17) |
| GitHub auth | ✅ 完了 | `gh auth status` | account: kenjiked / protocol: https / scopes: gist, read:org, repo, workflow |
| ArduPilot | ✅ 完了 | `ls ~/GitHub/ardupilot` | repo 存在、submodule 初期化済み、board=sitl |
| SITL | ✅ 完了(ビルド済) | `ls ~/GitHub/ardupilot/build/sitl/bin/arducopter` | arducopter バイナリ存在。起動テストは未記録（要実施） |
| MAVProxy | ✅ 完了 | `command -v mavproxy.py` | ~/.local/bin/mavproxy.py |
| pymavlink | ✅ 完了 | `python3 -c "import pymavlink;print(pymavlink.__version__)"` | 2.4.49 |
| dronekit | ✅ 完了 | `python3 -m pip show dronekit` | 2.9.2（`__version__` 属性は無し、pip show で確認） |
| Docker | ✅ 完了 | `docker --version` / `docker info` | Docker 29.6.0、daemon reachable |

> 補足: Python ライブラリはグローバル（`~/.local`、venv 未使用）にインストール。動作上は問題なし。venv 化は任意で後回し。

## GitHub Submission Preparation

- **repo name**: drone-app-flight-challenge
- **repo URL**: https://github.com/kenjiked/drone-app-flight-challenge
- **visibility**: public
- **initial commit**: "Initial project structure" (6c04bc1)
- **remote**: origin → https://github.com/kenjiked/drone-app-flight-challenge.git
- **branch**: master

## Assignment / Submission Notes

- GitHub repo URL 共有は可能（repo は public、push 済み）。
- Pull Request 提出が必要かは **要確認**。資料上は Day4「Pull Request 基礎」に含まれる可能性が高いが、
  今回の課題の提出形式が「自分の repo URL 共有」なのか「指定 repo への Pull Request」なのかは未確定。
- **要確認**: 提出形式（URL共有 / 指定repoへのPR）を講師に確認する。

### 講師への確認文面案

> 課題提出について確認させてください。GitHub リポジトリは作成済みです。
> 提出は自分のリポジトリ URL を共有する形でよいでしょうか？
> それとも、指定リポジトリに Pull Request を作成する形式でしょうか？
> リポジトリ: https://github.com/kenjiked/drone-app-flight-challenge

## Next Actions

1. SITL を起動確認する（`cd ~/GitHub/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter --console --map`）
2. `docs/sitl-test-log.md` に動作確認ログを追記する
3. 変更を commit / push する
4. 講師に提出形式（URL共有 / PR）を確認する
