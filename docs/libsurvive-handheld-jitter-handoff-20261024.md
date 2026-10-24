# WiVRn Codex: 新規較正成功後の手持ちジッター A/B

まず libsurvive を `f8886a2` 以降へ更新する。
`612629c` の native recorder 排他修正と `f8886a2` の加速度逆算修正を含む。
Monado の座標変換・履歴・予測・追加 clamp を変える必要はない。
実際にロードした core/plugin のパス・SHA と source revision を引き続き保存すること。

今回のユーザー実測は
`~/LH-Dump/2026-10-24_203705_libsurvive_907114938_814822`。
既存較正なしで正常に追跡できたとの確認がある。
右 Index の高速回転後に置き、Tracker を持ち上げて観察したとの説明と、
gyro の高速回転 84.37〜87.37 秒／Tracker の動き始め約 87.77 秒が対応する。
[解析結果](lighthouse-handheld-results-20261024.md)を参照。

## 記録の不具合

約 37.113 秒の native 2 行に NUL と混在した書き込みがある。gzip CRC は正常。
JSONL queue drop/unwritten=0 とは独立した同期 native writer の問題である。
新しい recorder では timestamp／本文、CONFIG、行列・共分散の全体を出力 mutex で保護する。
更新後も gzip EOF/CRC だけでなく、native の NUL と壊れたレコードを確認する。
この修正をジッター原因の解消と説明しないこと。

## 次の比較: 角速度プロセス雑音だけ変更

現時点で新しい既定値を決める変更はない。同じ更新済みバイナリで次の 2 条件を選べるようにする。

| 条件 | libsurvive へ明示的に渡す引数 |
|---|---|
| A: 標準 | `--process-weight-ang-vel 60` |
| B: 試験 | `--process-weight-ang-vel 0.6` |

既存の選択手段がなければ実測用の選択を追加する。
**これらを新しいデフォルトとして無条件に追加しないこと。**
選んだ条件、実際に渡した引数、native-options の実効値を manifest 等に残す。
recording が無効の通常起動時まで、試験設定を残さない。

両者とも以下を同じ値へ固定する。

- `kalman-minimize-state-space=0`
- `process-weight-gyro-bias=1e-8`、`process-weight-acc-bias=1e-8`
- その他の IMU・光学・静止判定・速度拘束・床追従設定
- 追加予測 0 ms。設定値だけでなく実効値も保存
- native raw IMU／calibrated IMU／光学／POSE_TIMING／tracking-state と既存 Monado 各 stage の記録

最初は、今回正常に作れた **libsurvive 自身の**較正を固定 snapshot にし、
各 A/B 試行用の別 config にコピーする。現在 live で更新されるファイルを毎回読み直さない。
SteamVR の基地局姿勢は使わない。A/B で状態次元、較正開始条件、source/build、記録量を混ぜない。
体感上良かった条件が見つかった後、別試行で新規較正からの挙動も確認する。

## ユーザー実測時の区間

A/B の順序を入れ替えて繰り返し、各開始・終了を区間マーカーへ記録する。
録画の wall-clock とネイティブ receive-clock を取り違えないこと。
マーカー入力が未実装なら、記録中にユーザーが押せる簡単な区間記録を用意する。
後から IMU の静かさだけで「手持ち」を決めない。

1. Tracker を台に固定して 10 秒以上静止。
2. Tracker を手に持ち、小さい自然な動きのまま 10 秒以上観察。
3. 数 cm のゆっくりした並進、数度の回転、動き始め・停止・逆転を各 5 秒程度。
4. Index の高速回転→台へ置く→静置。その後 Tracker を持ち上げて 2、3 を繰り返す。

主観評価は見た目のジッターだけでなく、動かし始めの遅れ、微小運動の消失、
停止後の滑り、速い運動からの復帰を別々に残す。
B は今回の再生で出力の細かい変動と gyro との内部不一致が小さくなったが、
真の運動を減衰させた可能性と live の遅れは未測定である。

## 残る調査

静置 Tracker の工場補正後 `I` の加速度ノルムは約 0.9404 g。
フィルタは残差 bias を学習済みなので、raw と residual bias 適用後を分けて評価する。
必要なら追加で、Tracker を異なる方向へ向けて各 5〜10 秒静止した記録を採る。
単一姿勢から bias／scale を決めて実動作を消す補正を加えないこと。

また、`survive_get_lighthouse_position` は補間・記録の副作用があり、simple API の
lighthouse getter は ctx lock ではなく poll mutex の下で呼ぶ経路がある。
今回の出力 mutex は gzip の排他を直す変更で、共有 BSD 状態全般の排他まで直すものではない。
この経路を調べる場合は、ctx→poll の既存 lock 順序を守ること。
poll mutex の内側で単純に ctx lock を追加するとデッドロックを作り得る。
この競合を今回の終盤ジッターの原因と断定できる証拠はまだない。
