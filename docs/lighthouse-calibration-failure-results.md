# 新規較正失敗の再生調査（2026-10-24）

最終実装は `4def8c2`。目的は異常な較正の再現と診断の修正であり、ジッター改善の確認ではない。
[WiVRn への返答・次の実測手順](libsurvive-calibration-diagnostics-handoff-20261024.md) と
[機械可読な結果・入力 SHA・ロード先証跡](lighthouse-calibration-failure-results.json) を参照。

## 固定した 12 条件

19:34（約 79.5 秒）・19:36（約 127.7 秒）の native 生入力を全長再生。
入力から `LH_POSE` を除外し、SteamVR の較正は使用していない。
poser/GSS は直列、再生速度制限なし、`report-in-imu=1`、追加の自動床追従なし。
バイアス Q は accel/gyro とも 0 または 1e-8、状態次元は両者とも完全なモデルに固定した。
各試行の初期 config／記録／出力は評価専用のファイルである。
良好初期値は 19:54 実測の固定 initial snapshot、不良初期値は 19:36 の initial snapshot。

| 入力 | 初期較正 | Q（両 bias） | native pose 数 | 原点距離最大 m | 100 mm 超の隣接 step 数 | reset 数 |
|---|---|---:|---:|---:|---:|---:|
| 19:34 | 不良 | 0 | 54,024 | 22.645 | 551 | 2 |
| 19:34 | 不良 | 1e-8 | 54,080 | 22.265 | 522 | 2 |
| 19:34 | 新規 | 0 | 33,051 | 1.237 | 5 | 0 |
| 19:34 | 新規 | 1e-8 | 33,051 | 1.238 | 5 | 0 |
| 19:36 | 不良 | 0 | 79,052 | 25.659 | 1,870 | 14 |
| 19:36 | 不良 | 1e-8 | 78,733 | 28.051 | 1,601 | 13 |
| 19:36 | 新規 | 0 | 80,590 | 1.146 | 4 | 0 |
| 19:36 | 新規 | 1e-8 | 80,590 | 1.144 | 5 | 0 |
| 19:34 | 良好（固定 snapshot） | 0 | 58,038 | 1.461 | 2 | 0 |
| 19:34 | 良好（固定 snapshot） | 1e-8 | 58,038 | 1.461 | 2 | 0 |
| 19:36 | 良好（固定 snapshot） | 0 | 93,728 | 1.512 | 3 | 0 |
| 19:36 | 良好（固定 snapshot） | 1e-8 | 93,728 | 1.511 | 3 | 0 |

step は receive-clock の間隔が `0 < dt <= 20 ms` のペアで集計し、無効姿勢をまたがない。
物理的な速い動作や初期較正の座標変更も含み得るため、すべてをジッターと呼ばない。
原点距離も世界座標の取り方に依存し、位置精度／RMSE ではない。
実測と再生ではスケジューリング、自動床追従、較正の更新タイミングが異なる。

不良初期値では Q=0 でも発散と reset を再現した。一方、新規と良好初期値では reset はなかった。
新規較正の **最初の実測失敗そのもの**は直列／実時間 threaded 3 回の再生では再現していない。
新しい診断を加えた最終版と従来版は、上記 12 条件それぞれで native POSE 行の SHA-256 が一致した。
診断追加によって追跡が改善したとの主張はしない。

初期の探索では live に更新される良好 config を試行ごとにコピーしていたため、
一部の比較で floor offset が一致していなかった。これらは同一出力の比較から除外し、
固定 snapshot で対照・最終版の双方を再実行した。JSON の
`controlled_output_pairs` と `excluded_from_pairwise_comparison` が採否の明細である。
棄却した半球ガードと初期値変更の試作も JSON に残すが、最終版には含めていない。

## 追加実測 19:54

正しい設定パスと良好な自己較正を使用した約 199 秒の native 記録も全長読み取った。
147,180 native pose、GSS 成功 18 回、tracking reset 0 回。
手持ち微小運動の時刻ラベルはないため、体感ジッター低減の判定には使っていない。
この記録については原記録の監査のみで、新コードによる再生は行っていない。

## 検証・範囲

関連 CTest 15/15、新しい重力・残差回帰テストの ASan/UBSan 成功。
旧ライブラリで新しい残差テストを動かすと、期待ノルム 200 に対して約 100 となり失敗する。
試作ガード版の不良入力全長 ASan 再生は 300 秒で timeout。成功として数えない。
新コードで LH-Dump 全量を再評価した結果ではない。

重力角は装置の設置角ではなく、局所重力と推定姿勢の整合性であり、急角度の BS を除外しない。
角度に基づく合否判定は導入していない。
新しい `GSS_QUALITY` は光学残差と物体の重力角を分け、`GSS_LIGHTHOUSE_GRAVITY` は BS ごとに
角度を出す。失敗 status の候補も記録する。詳細は
[記録形式](lighthouse-tracking-state.md) を参照。

## 再集計

評価用の個々の記録とコマンドは `.lh-evaluation/calibration-failure-2026-10-24/` に保持している。
次のツールは成功終了した run.json と record.rec.gz を集計し、原記録の SHA、pose step、
GSS ログ、valid/invalid ごとの callback 時刻差を出す。原データを変更しない。

```sh
python3 tools/lighthouse_calibration_audit.py .lh-evaluation/calibration-failure-2026-10-24 \
  --source ~/LH-Dump/2026-10-24_193444_libsurvive_112556333_758814 \
  --source ~/LH-Dump/2026-10-24_193640_libsurvive_225267645_761146 \
  --source ~/LH-Dump/2026-10-24_195448_libsurvive_111892475_777039 \
  --baseline-build-proof .lh-evaluation/jitter-2026-10-24/bias-walk/build-proof.json \
  --prior-corpus-gravity-audit .lh-evaluation/calibration-failure-2026-10-24/old-corpus-gravity-audit.json \
  --output /tmp/lighthouse-calibration-audit.json
```

出力には探索・試作も含まれる。固定初期値での比較対象は公表 JSON の `controlled_output_pairs` に従う。
