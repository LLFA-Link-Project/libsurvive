# LH-Dump のブラウザ可視化

`tools/lighthouse_viewer.py` は、保存済みの姿勢を 3D と時系列で確認する
ローカル HTTP サーバーです。Python 3.9 以上の標準ライブラリだけで動き、
ブラウザ側にも外部 CDN や追加パッケージは必要ありません。

```sh
cd ~/libsurvive
./tools/lighthouse-viewer.sh 8765
```

同じ LAN の PC から `http://サーバーPCのIPv4アドレス:8765/` を開きます。
この環境のアドレスは `http://192.168.0.37:8765/` です。
指定したポートで前面起動し、Ctrl-C で終了します。読み取り中のキャッシュ作成も中止します。
常駐化・自動起動登録はしません。
ポート番号を省略すると 8765 になります。

別のポートや保存先、この PC からだけの接続を指定する場合:

```sh
./tools/lighthouse-viewer.sh 8080
./tools/lighthouse-viewer.sh 8080 --dumps /path/to/LH-Dump
./tools/lighthouse-viewer.sh 8080 --host 127.0.0.1
```

Python を直接起動することもできます。こちらの `--host` の既定値は `127.0.0.1` です。

```sh
python3 tools/lighthouse_viewer.py --dumps ~/LH-Dump --host 0.0.0.0 --port 8765
```

これはデータを閲覧するための LAN 用サーバーで、認証はありません。
公開するのは専用画面と記録の読み取り API です。ホームディレクトリの
ファイル一覧や任意パスを配信するサーバーではありません。

## 区間を見て、コメントと一緒に渡す

1. 左の記録・デバイス・記録段階を選びます。日付や `libsurvive` で検索できます。
   初回だけ原ファイルを最後まで読み、検索用キャッシュを作成します。
2. 開始／終了秒を指定して「この区間を開く」を押します。最大 60 秒です。
   「記録全体から移動」や動作マーカーでも移動できます。
3. 低速再生、実サンプル単位のコマ送り、位置・回転の拡大で見ます。
   3D はマウスで回転・ズームでき、「軌跡全体を収める」で固定視点を合わせます。
   「区間先頭のデバイス軸」は先頭姿勢で固定した軸です。
4. 気になる範囲をグラフ上でドラッグするか、下の範囲欄で指定します。
   「現在位置を開始に／終了に」も利用できます。
5. 観測コメントを入力し「区間・コメント・リンクをコピー」を押し、チャットへ貼ります。

コピーにはダンプ名、ストリーム ID、元ファイル全体の SHA-256、serial、記録段階、
参照点・座標系、時刻の基準、選択秒、読み込んだ範囲内のセンサー tick / Hz、
表示倍率・軸・補間・視点・再生速度、コメント、再現 URL と JSON 状態を含みます。
URL の fragment に保存するため、コメントをサーバーへ保存・送信しません。
SHA-256 またはデバイスが一致しない共有リンクはエラーになります。
HTTP の LAN 接続で Clipboard API が使えない場合にも従来のコピー処理を試み、
それも利用できなければ手動コピー欄を開きます。

例えば「`2026-10-25_211424_…` の C6 tracker、263.5–265.5 秒、
デバイス軸の X 方向へ滑って戻る」とコメントできます。
選択範囲と読み込み範囲を別々に保存し、先頭原点が変わって見え方が変わることを避けます。
リンクのホスト部分を `127.0.0.1` から LAN IP へ変更しても復元状態は同じです。

「A の滑りを見る」は 10/25 の A 原記録の 260–266 秒を開きます。
「変更前後」は同じ A 入力を処理した `baseline` と `tracker-final` を並べます。
これはオフライン出力の比較で、実機での改善を証明する表示ではありません。
「最新の記録」は現在のカタログ中で日付が最新の native 記録です。
新しいダンプを追加したら「記録一覧を更新」を押してください。

## 読むデータと規模

`--dumps` 直下の `manifest.json` があるセッションを対象にします。
`native.rec[.gz]` と `poses.jsonl[.gz]` を別ストリームとして扱います。
libsurvive の Monado JSONL は `backend_received` / `history_input` /
`app_output` / `hand_output`、input、参照点、座標系ごとに分離します。
native の CONFIG から serial を決め、WM 番号が他のデバイスへ割り当て直されたら
別トラックにします。

この checkout に既存の評価データがあれば、次も一覧に追加します。

- `.lh-evaluation/handheld-fix-2026-10-25/` の A / R4 の変更前後 4 出力。
- `.lh-evaluation/full-2026-10-24/replay-*/` の `*-0.rec.gz`。
  これらは過去の core による出力で、現在の core の再評価とは扱いません。

2026-10-26 時点の環境では 80 ダンプ・164 ストリームを列挙しました。
一覧への登録は全件の読み取り完了や品質確認を意味しません。
選択したファイルごとに最後までストリーム読み取りし、SQLite へ保存します。
USB pcap 全体の展開や libsurvive の再実行は行いません。
すでに記録・生成済みの姿勢を見る仕組みです。

原記録を書き換えず、派生キャッシュだけを
`.lh-evaluation/viewer-cache/` に作成します。場所は `--cache` で変更できます。
キャッシュキーは元の絶対パス・サイズ・mtime_ns・パーサーバージョンです。
インデックス作成中に元ファイルが変わった場合は公開しません。
一度に 1 ストリームずつ処理し、メモリへ記録全体を読み込みません。
ブラウザへの送信は 1 トラック・最大 60 秒・10 万点以内で、
サンプルを間引かず、位置・診断値は元の double 精度を保って JSON で送ります。
quaternion は表示用に正規化します。
初回の待ち時間とキャッシュ容量は記録の長さに比例します。
キャッシュは削除して作り直せます。

CRC / gzip 終端エラーは `readable-prefix` と表示し、完全な行までを表示します。
NUL・壊れた行は数を報告します。数値や quaternion が使えない既知の姿勢は
無効サンプルとして扱います。解析不能な行の発生位置を常に特定できるわけではありません。
CRC の成否と manifest の `recording` / `complete` は別々に表示します。
CRC が通っても入力の無損失、較正の正しさ、追跡精度の証明にはなりません。

## 姿勢・時刻・表示の定義

- native は `POSE_STATE` があれば床補正前の IMU 原点の高精度状態を使います。
  なければ `POSE` を使い、床・profile の補正が含まれる可能性を表示します。
  `POSE_TIMING` / `POSE_STATE` の full tick と Hz はそのまま残します。
  native の quaternion は wxyz です。
- JSONL の xyzw を wxyz へ並べ替えます。SteamVR は記録された
  world-from-driver / driver-from-head を合成し、chaperone / stabilization 前の
  参照姿勢として表示します。Monado の各 stage は記録値をそのまま使います。
  3D カメラの向きだけ Y-up / Z-up に対応します。
- 時間軸は native の受信秒、JSONL は clock_sync 先頭からの callback
  `CLOCK_MONOTONIC` 秒です。clock_sync が読めなければ最初の callback を原点にし、
  そのことをメタデータに記載します。二つの時計を自動同期しません。
  マーカーは clock_sync と native INPUT_TIME 冒頭 32 標本の epoch 推定を使って
  表示秒へ変換します。入力から表示までの遅延計測ではありません。
- native の gyro / bias は最新 POSE_STATE の full tick と一致する POSE_BIAS、
  その tick の下位 32 bit が一致する calibrated `I` だけを結びます。
  センサー時計の差が計算できるサンプルでは、20 ms の接続判定にその差を使います。
  それ以外は callback / 受信秒の差を使います。
- 20 ms 超のギャップ、無効姿勢、センサー時刻の逆行をまたぐ線・補間は描きません。
  平滑化しません。既定は実サンプルを保持し、任意で線形 / shortest-hemisphere
  quaternion slerp 補間を選べます。表示更新はブラウザのフレームレートに従うため、
  高周波成分は全サンプルを含む軌跡・グラフと低速再生で確認してください。
- 位置の原点は各表示区間の最初の有効サンプルに固定します。A/B は独立した原点です。
  3D の箱は実機のモデル・寸法ではなく参照点を中心とした模式図です。
  位置倍率と回転倍率は 3D だけに適用し、グラフには適用しません。
  グラフの回転は先頭姿勢からの shortest quaternion log、位置・回転の軸は選択した
  固定 world / 先頭デバイス軸です。180 度近辺では log の符号が切り替わり得ます。
- `step mm` は接続できる隣接サンプルの位置差です。RMS 誤差や速度ではありません。
  gyro は `|calibrated gyro − estimated bias|`、静止判定時間はフィルター内部値です。
  未記録の指標はゼロに置き換えず「未記録」と表示します。

手の実際の動きも含み、SteamVR も推定値です。
異なるセッションの軌跡を同じ物理動作の誤差として扱わず、
実機の装着感や rendering / prediction の評価には実機比較を併用してください。
記録形式の出典は `~/WiVRn/docs/lighthouse-recorder.md` と
`~/WiVRn/docs/libsurvive-recording.md` です。

## 検証

```sh
python3 -m unittest discover -s tools -p test_lighthouse_viewer.py -v
node --check tools/lighthouse-viewer/viewer.js
```

13 ケースで full tick の保持、IMU / bias の対応、POSE fallback、serial の再割当、
gzip 切断、NUL / 非有限状態、stage / profile の分離、xyzw と Valve 合成、
無効姿勢 / partial validity、範囲上限、キャッシュ更新、中止時の未完成データ非公開を確認します。

実ブラウザの検証は任意の Playwright / Chromium が必要です。
実データ A と変更前後のキャッシュがある環境で、起動中のサーバーに対して実行します。

```sh
npm install --prefix /tmp/lh-viewer-test playwright
PLAYWRIGHT_BROWSERS_PATH=/tmp/lh-viewer-browsers \
  /tmp/lh-viewer-test/node_modules/.bin/playwright install chromium
PLAYWRIGHT_MODULE=/tmp/lh-viewer-test/node_modules/playwright \
PLAYWRIGHT_BROWSERS_PATH=/tmp/lh-viewer-browsers \
LH_VIEWER_URL=http://192.168.0.37:8765 \
  node tools/test_lighthouse_viewer_browser.cjs
```

実サンプル数・SHA、3D 描画、変更前後、コマ送り、HTTP コピー、共有状態の往復、
SHA 不一致、任意パスの拒否、60 秒超の拒否、再生、390 px 幅を確認します。
スクリーンショットは `/tmp/lh-viewer-browser/` に出ます。
