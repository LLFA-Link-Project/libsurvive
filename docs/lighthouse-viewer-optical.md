# ビューワーの光学解と周辺デバイス

融合後の姿勢と、融合前の光学ソルバー候補を同じシーンに表示できます。
`--show-raw-obs --report-in-imu` で記録された
`WMn-raw-obs EXTERNAL_POSE` を読み、CONFIG の serial へ結びます。
任意の EXTERNAL_POSE や SteamVR の出力を光学解として扱いません。

光学解は MPFIT などの候補姿勢です。融合側で採用されたことを保証する値ではなく、
初期化には IMU・直前の推定姿勢も使われます。独立した真値ではありません。
センサー計測 tick が記録されていないため、受信秒で重ねます。
表示中の融合位置との差には、受信時刻の差と動いていた距離も含まれます。
光学解を補間せず、受信から 50 ms を超えたら現在位置のマーカーを消します。
軌跡は 20 ms 超のギャップで切ります。

## 未保存の光学解を調べる

元記録に候補姿勢がない場合、HTTP サーバー内で追跡処理を実行しません。
明示的なオフライン処理として、入力だけの native 記録と、その記録で使った
**libsurvive 自身の較正**を使って診断用の再生を作成します。
元記録や元の較正を書き換えません。作業用 config だけが出力ディレクトリ内で更新されます。

この環境の A は次のコマンドで作成しました。

```sh
python3 tools/lighthouse_optical_replay.py \
  --cli .lh-evaluation/handheld-fix-2026-10-25/baseline-bin/survive-cli \
  --input .lh-evaluation/handheld-fix-2026-10-25/A-input.rec.gz \
  --calibration ~/LH-Dump/2026-10-25_211424_libsurvive_158643023_1366788/calibration.initial.json \
  --target-id dump:2026-10-25_211424_libsurvive_158643023_1366788:native \
  --target-file ~/LH-Dump/2026-10-25_211424_libsurvive_158643023_1366788/native.rec.gz \
  --output .lh-evaluation/viewer-optical/A-baseline
```

出力先は未作成のディレクトリを指定します。同じ出力先を上書きしません。
既存の A-input は、A の読める範囲から作成済みの CONFIG / 入力イベントの記録です。
他の記録でも入力専用ファイルを用意してください。元の POSE / 外部姿勢 /
LH_POSE / OPTION をそのまま入力へ混ぜると、過去の出力や設定を再注入することになります。
再生は指定した CLI の設定を使うため、再現性が必要な場合はライブラリ・plugin も含めて
固定したビルドを指定します。

GSS・自動床補正・threaded poser を無効にし、IMU 参照点で raw-obs と
POSE_STATE を記録します。この条件を元の live 実行と同一だとは扱いません。
光学候補も追跡状態・処理順によって変わるため、元の live 光学解の復元を保証しません。
較正を最初から求め直した出力や別の world 座標系へ自動整列する機能ではありません。

`viewer-source.json` が完成すると、サーバーの「記録一覧を更新」または再起動で
`optical diagnostic replays` に追加されます。元記録には、ID と元ファイル SHA-256 が
一致した場合だけ自動的に重ねます。派生ファイルも生成時の SHA-256 と照合します。
参照点・world フレーム・serial が一致しない候補は重ねません。
UI に「再生で求めた光学解」と表示し、元記録に直接保存されていた候補と区別します。

共有テキストには候補ファイルの ID / SHA、入力・較正・CLI の SHA、再生コマンドと
条件・制限も含みます。リンクにも候補の SHA を持たせ、不一致なら復元を拒否します。
較正ファイルや候補の記録そのものを URL に埋め込むことはありません。

## A 全体の生成結果（2026-10-26）

固定した旧設定 baseline ビルドで約 37.3 秒、終了コード 0。
派生 gzip は終端まで読め、破損行 0 です。位置は再生から求めた値です。

| デバイス | 光学候補数 | 候補の受信秒 |
| --- | ---: | --- |
| Index Left / LHR-F39A0C97 | 27,450 | 1.850545–305.327237 |
| Index Right / LHR-DF24384D | 29,436 | 2.032546–310.951049 |
| VIVE Tracker / LHR-C6F86C46 | 31,817 | 42.475144–310.951095 |

- 元 A: `b3ec6f2a0b4d2fb76cc72eeccf04041801986223fe74e589e363ba1b48329f01`
- 入力: `20b9d8ce26c4046b2ffb7e62293c433f757f5d8d9b769707c8f61ad19f600f7f`
- libsurvive 較正: `c0fd8d7485628f0471b3001e13786cf092bfb06ce66bd384d0740188087dd9da`
- 派生記録: `cf7691b456a7a43c0ff880c9473529aea901842a3c204dc2c03fd609c8d78106`

212.683011–218.683011 秒の Index 静置時の異常と、234.176186–240.176186 秒の
Tracker 持ち上げ、260–266 秒の手持ちのいずれも、この候補記録で確認できます。
光学候補がない元ダンプ全件を再生成したわけではありません。

## シーン API の定義

`/api/window` は選択トラックの rows に加えて、`context` と `optical` を返します。
周辺デバイスは同じ記録・stage・参照点・world フレームのものだけを選びます。
別 stage の grip / palm を重複したデバイスとして表示しません。
範囲内の行とその直前の 1 行を取得し、カーソルより後の姿勢は保持値に使いません。

native の `LH_POSE` はモード／channel と ID ごとに時系列で保持します。
最後に記録された値を全時刻へ使い回しません。これは床補正後の位置なので、
初期の libsurvive config と直前の POSE_STATE で分かる floor-offset を戻してから
床補正前の IMU world と重ねます。床補正が不明な BS は raw IMU シーンへ重ねません。
初期較正もキャッシュ依存ファイルに含めます。

CONFIG の model_number / device_class は機種別の模式図に使います。
これは IMU 参照点・軸に置くアイコンで、factory sensor geometry や CAD モデルではありません。
JSONL では記録された role / device_class で種類を判定します。

1 トラック 10 万行、周辺 32 トラック、合計 30 万行、最大 60 秒で制限します。
`anchor=first` は選択トラックの最初の有効姿勢、`anchor=<受信秒>` はその時刻の
有効な実サンプルを原点として返します。読み込み範囲外でも固定原点を再取得できます。
SQLite 接続はリクエストごとに閉じます。
