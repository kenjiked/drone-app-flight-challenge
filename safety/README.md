# safety/ — ③守る層の実装

設計は [../docs/safety-design.md](../docs/safety-design.md)。ここは機体側(ArduPilot)の安全機構。

| ファイル | 層/Step | 内容 |
|---|---|---|
| `patrol_safety.parm` | Step1(設定) | 標準フェイルセーフ+ジオフェンス。異常→RTL/LAND を設定だけで成立 |
| `patrol_safety.lua`  | Step2(Lua)  | 自作の離陸前チェック(aux-arm)＋飛行中監視(姿勢/電池/範囲→RTL) |

## SITL での有効化手順

```bash
# 1) スクリプトを SITL の scripts/ に置く（例）
mkdir -p ~/GitHub/ardupilot/scripts
cp safety/patrol_safety.lua ~/GitHub/ardupilot/scripts/

# 2) SITL を起動して、設定を流し込む（MAVProxy から）
#    param load でまとめて設定 → 反映のため再起動
param load /full/path/to/safety/patrol_safety.parm
reboot
```

起動後、GCS に `PatrolSafety 0.1 loaded` が出れば Lua 有効。
未アーム中に電池不足/位置未確定なら `みまわり: …` の理由付きで**離陸がブロック**される。
飛行中(AUTO)に姿勢/電池/範囲が閾値を超えると**自動で RTL** に切り替わる。

> 注: 機体側(ここ)は「電池・姿勢・GPS・範囲」といった**機体が知っている状態**を守る。
> DID(人口集中地区)や建物高さなど**地図・法令の情報に基づく離陸前判定**は、
> 計画側(Webバックエンド `src/` の飛行前チェック)で行う。役割を分けている。
