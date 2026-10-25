# WiVRn Codex: 手持ち微小運動向けの追跡変更を反映する

libsurvive の手持ち追跡の変更を取り込み、core と全 plugin を同じソースから再ビルドする。
実装は `7efb618`、controller / HMD への適用を既定から外した変更は `b769138`。
**`b769138` 以降をまとめて取り込む。** 前者だけの状態を実測版にしない。
新しい既定値は汎用 Tracker の静止拘束の入り方を変える。Index と HMD は従来の拘束を保つ。
[実装と再生結果](lighthouse-handheld-tracking-results-20261025.md)を参照。
内部 tracker 構造体にもフィールドを追加しているため、旧 plugin と新 core を混ぜない。

## 反映する設定

角速度プロセス雑音は **60** にする。以前の B=0.6 はユーザーが不安定と評価しており、採用しない。
追加予測は引き続き設定・実効値とも 0 ms に固定する。
Monado の座標変換や履歴への変更は必要ない。

新しい libsurvive の通常設定は以下。

```text
process-weight-ang-vel=60
kalman-minimize-state-space=0
process-weight-acc-bias=1e-8
process-weight-gyro-bias=1e-8
kalman-zvu-stationary-ramp=0.5
kalman-zvu-stationary-gyro-scale=0.02
kalman-zvu-stationary-velocity-sigma=0.1
kalman-zvu-motion-guard-time=0.5
kalman-zvu-motion-all-devices=0
kalman-align-observation-quaternion=0
```

`kalman-zvu-motion-all-devices` と最後の quaternion オプションは試験用で、今回は有効にしない。
controller まで拘束緩和を広げると A の一部で跳びが増えたため、今回は Tracker に対象を絞った。
同じ向きを表す符号と共分散の整合性を直す候補だが、A の一部の Index の跳びが増えた。
光学 innovation の一般的な制限も今回の採用版には入っていない。

比較用に旧拘束へ戻す場合だけ、同じ新バイナリで次の 2 値を変える。

```text
kalman-zvu-stationary-ramp=0
kalman-zvu-stationary-gyro-scale=0
```

バイナリ、較正、状態次元、記録量、他の設定を揃える。
これは旧「B=0.6」と異なる比較なので、ラベルを明確に分ける。
最初は正常に作れた libsurvive 自身の immutable calibration snapshot を各試行へコピーする。
SteamVR の基地局姿勢は使わない。動いている live 設定ファイルを各回の開始条件にしない。
固定較正で確認した後、新規較正からも別試行で確認する。

## 見てほしい動作

1. Tracker を台へ置いて 10 秒以上。静置時の安定性と、置いてから落ち着くまでを見る。
2. 手に持ったまま、小さい自然な動きで 10 秒以上。滑り・戻りを観察する。
3. 回転させず、数 mm〜数 cm のゆっくりした並進、停止、逆転。微小運動が消えないかを見る。
4. 小さな回転を伴う動き。向きを変えた前後で滑りの方向も記録する。
5. 右 Index を高速回転して置き、その後 Tracker を持つ。Index の復帰と Tracker の手持ちを別々に評価する。

各開始と終了をマーカーへ保存し、旧拘束／新拘束の順序も記録する。
見た目のジッターに加え、動き始めの遅れ、意図した小さな動きの消失、停止後の滑りを分けて残す。
合成試験では 2 mm の微小運動の追従が改善したが、実機での体感改善はまだ確認していない。
Index の大跳びが解消したと説明しない。

## 記録と終了

既存の raw/calibrated IMU、光学、POSE_TIMING、tracking-state、Monado 各 stage を使う。
新しい native record 種類を追加していないため、ロガー形式の変更は不要。
新オプションの実効値が native OPTION に残ること、実ロードした core/plugin のパス・SHA と
source revision、追加予測の実効値を引き続き保存する。
serial で Tracker と左右 Index を識別し、WM 番号を記録間で固定と仮定しない。

A の終了ハングはこの変更で解消を確認したものではない。
再び終了が止まった場合、SIGKILL の前に全スレッドのスタックを採取する。
gzip 終端や最終 queue 統計のない記録を complete に書き換えない。
