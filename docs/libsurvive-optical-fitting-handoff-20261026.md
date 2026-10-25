# WiVRn Codex: 光学フィットの変更を実測する

libsurvive の今回の光学フィット変更を取り込み、core と全 plugin を同じソースから再ビルドする。
core は `e6e9707` の実装と、全件評価後に既定無効へ戻した **`32848dc` 以降をまとめて取り込む**。
`e6e9707` 単独を実測版にしない。
光学の計測時計を保存する診断追加は `24f11b0`、光学残差の対応修正は `93fccbd`。
新しい optimizer 構造体を使うため、旧 plugin と新 core を混ぜない。
WiVRn や Monado の作業ツリーはこのタスクからは変更していない。

[再生結果・限界](lighthouse-optical-fitting-20261026.md) と
[数値・入力／ビルドの証拠](lighthouse-optical-results-20261026.json) を参照。
A の手持ち区間では光学位置の局所残差 p95 が1.131 → 0.628 mmへ減った。
これは内部の連続性指標であり、実機の体感・真値との精度・遅延改善はまだ確認していない。
Index の跳びや不良較正からの復帰を解決したとは扱わない。

## 同じバイナリで A→B→A

既定値は `mpfit-light-huber=0`、`mpfit-light-outlier-ratio=0`。
全件評価で位置の参照差が増える記録が残ったため、Huber は試験時だけ明示して有効にする。
汎用 Tracker の通常追跡だけへ、初期残差から閾値を選ぶ Huber 損失を掛ける。
controller/HMD の残差や初期較正・GSSの目的関数への適用、出力平滑化、追加予測は加えない。
ただし較正・追跡の連動による他デバイスへの間接影響は、USB 再生で残った。

最初は正常な **libsurvive 自身の immutable calibration snapshot** を毎回コピーし、
同じバイナリ・記録量・予測・他設定のまま、次だけ切り替える。

```text
A: mpfit-light-huber=0
B: mpfit-light-huber=-1
A: mpfit-light-huber=0
```

`mpfit-light-outlier-ratio` は全試行0。6などへ変更しない。
Index の一部で大幅に効いた受光除外の実験だが、別区間・他記録で悪化したため既定に採用していない。
新しい負値モードは固定3 mradと異なり、初期残差が大きい場合に重み付けを弱める。
以前の「B=角速度プロセス雑音0.6」と混同しないラベルを使う。

従来の採用設定を維持する。

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

追加予測は設定・実効値とも0 ms。Valve の基地局較正を使わない。
Monado の座標変換・履歴を同時に変更しない。
正常な固定較正での比較後、新規較正からの別試行も行い、較正成否と手持ち時の品質を分けて残す。

## 動作とマーカー

1. 台上で10秒以上。置いた後の収束と静置状態を確認。
2. 手に持って自然な微動を10–20秒。滑って戻る動き、向きを変えた際の滑る方向を確認。
3. 回転を抑えて数 mm〜数 cm の並進、停止、逆転。小さい動きの消失と動き始めの遅れを確認。
4. 小さい回転を伴う動き、ゆっくりした回転と並進。
5. Index の高速回転後に置き、Tracker を持ち上げる。Index の復帰と Tracker の手持ちを別々に評価。
6. 短い遮蔽から復帰し、姿勢の跳び・出力の途切れ・引きずりを確認。

各動作と A/B の切替をマーカーで残す。完全静止時の RMS だけで判断しない。
BS 1.0 / 2.0 は利用可能な構成で別試行にする。急角度の設置を禁止する条件は追加していない。

## ロガー

既存の raw/calibrated IMU、光学入力、POSE_STATE / POSE_BIAS / POSE_TIMING、
Monado の backend_received / history_input / app_output などを維持する。
`show-raw-obs=1` を各試行で揃えて、融合前の光学候補も保存する。

今回 `show-raw-obs` 有効時には、次の診断行が直後の `EXTERNAL_POSE` と一組で出る。

```text
<receive-seconds> WMn-raw-obs OPTICAL_TIMING <full-64-bit-tick> <device-Hz>
<receive-seconds> WMn-raw-obs EXTERNAL_POSE <x y z qw qx qy qz>
```

native は既存の書込経路を使うため、Monado の新しい出力形式は不要。
タグ検査・要約が既知タグだけを数えている場合は `OPTICAL_TIMING` を追加する。
これは光学観測の sensor clock であり、次の光学候補にだけ使う。
直前の POSE_STATE の時計や、単純APIの時刻で代用しない。
`report-in-imu=1` を診断条件として揃え、参照点が頭／gripのときにはgyroとの直接突合を行わない。

実ロードした core/plugin のパスと SHA、ソース revision、初期／終了較正 snapshot、
新設定の実効値・追加予測0 ms、入力drop・writer drop・終了状態を引き続き記録する。
未対応の診断があることを0や正常と見なさない。
元記録は上書きせず、較正失敗を含めた完全な記録として残す。

## 測定結果の返却

良い／悪い両区間を、ビューアーの「区間をコピー」で共有してほしい。
ダンプ名、SHA、serial、範囲、表示時刻、観察コメントにより同じ実標本へ戻れる。
native の光学候補があれば再生成せずそのまま表示できる。
今回のローカル A 前後は「光学フィットの変更前後」で確認できる。
HTTP サーバーを常駐化しない。必要な時だけ `./tools/lighthouse-viewer.sh 8080` で起動する。
