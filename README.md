# Unitree LiDAR 自走ロボット (ROS 2 Humble)

Unitree L1 / L2 LiDAR と Point-LIO で自己位置を求め、Nav2 で経路を計画し、Arduino でアッカーマン (前輪操舵) の
RC カーを走らせるためのリポジトリです。スマホのブラウザからモードを選び、地図をタップして目的地を送れます。

> ROS 2 のコマンドの使い方は **[ROS 2 コマンド チートシート](docs/ros2-cheatsheet.md)** にまとめてあります。

もとは Docker イメージ用のリポジトリでしたが、今はロボットに直接入れて使う形を基本にしています。
Docker イメージ (sim 含む) は[最後の節](#docker-で動かす-任意)で引き続き作れます。

## 目次

* [使い方の流れ](#使い方の流れ)
* [中身](#中身) / [セットアップ](#セットアップ-ubuntu-2204--raspberry-pi) / [設定 (`robot_config`)](#設定-robot_config)
* [スマホでモードを選んで起動する](#スマホでモードを選んで起動する-robot_launcher) /
  [スマホから地図をタップして目的地を送る](#スマホから地図をタップして目的地を送る-robot_map--robot_goal-web)
* [コマンドで起動する](#コマンドで起動する-robot_bringup--robot_goal) (Arduino を挿したら自動で起動する設定を含む)
* [LiDAR から Arduino までの仕組みと調整](#lidar-から-arduino-までの仕組みと調整)
* [走行キャリブレーション](#走行キャリブレーション-robot_operation-calibrate_drive)
* [Point-LIO だけを動かす](#point-lio-だけを動かす) / [Point-LIO のバージョン](#point-lio-のバージョンについて)
* [シミュレーション (Gazebo)](#シミュレーション-gazebo) / [Docker](#docker-で動かす-任意)

## 使い方の流れ

1. **入れる**: ロボット (Raspberry Pi など) で `./scripts/setup-raspi.sh l1` を実行する ([セットアップ](#セットアップ-ubuntu-2204--raspberry-pi))。
2. **ロボットの設定を書く**: `src/common/robot_config/config/robots/rc_car.yaml` に LiDAR の取付と
   ポート (`/dev/serial/by-id/...`) を書く。取付角度と地面の高さは `ros2 run lidar_bridge ground_check` で確かめる
   ([設定](#設定-robot_config)、[仕組みと調整](#lidar-から-arduino-までの仕組みと調整))。
3. **自動で起動するようにする**: `./scripts/install-autostart.sh l1` (Arduino を挿すとモード選択の画面が起動する)。
4. **走行キャリブレーション**: スマホで `http://<IP>:8080` を開き、「キャリブレーション」を選んで計測し、結果を「使う」。
5. **走らせる**: 「目的地設定の自走」を選び、地図をタップして目的地を送る。

## 中身

| 場所 | 内容 |
| --- | --- |
| `src/common/robot_config/config/` | **全ノードの設定** (共通の `common.yaml` と、ロボットごとの `robots/<名前>.yaml`)。[下記](#設定-robot_config) |
| `src/common/` | 全機種共通のパッケージ (下の表) |
| `src/l1/`, `src/l2/` | LiDAR の機種ごとのパッケージ (ドライバなど) |
| `src/sim/` | Gazebo のシミュレーション (`sim_gazebo`) |
| `firmware/` | Arduino のスケッチ (`operator.ino`) |
| `scripts/` | セットアップ (`setup-raspi.sh`)、外部リポジトリの取得 (`fetch-external.sh`)、自動起動など |
| `patches/` | 外部リポジトリに当てる、こちらの変更 |

### このリポジトリのパッケージ (`src/common/`)

| パッケージ | 内容 |
| --- | --- |
| `robot_config` | 全ノードの設定をまとめて持ち、launch が起動時に各ノードへ配る |
| `lidar_bridge` | LiDAR ドライバのトピックを Point-LIO 用の名前・座標に付け替える。停止判定 (ZUPT 用)、取付角度の確認 |
| `obstacle_detector` | 点群から Nav2 の障害物とクリア用の点群を作る (坂の判定つき) |
| `robot_navigation` | Nav2 の起動と設定 |
| `robot_operation` | Nav2 の速度指令を Arduino のサーボ角・スロットルに変換。走行キャリブレーション |
| `robot_map` | Point-LIO の点群からスマホ表示用の 2D 地図を作る |
| `robot_goal` | 目的地を送る (コマンド、スマホの Web 画面) |
| `robot_bringup` | 全部を 1 つのコマンドで起動する |
| `robot_launcher` | スマホでモードを選んで起動する画面 |

### 外部リポジトリ (git サブモジュール)

外部のリポジトリは取り込まず、git のサブモジュールとして取り込み元のコミットを参照します。
こちらで手を入れたものは `patches/` に変更を置き、取得のときに当てます。

| 場所 | 取り込み元 | コミット | こちらの変更 |
| --- | --- | --- | --- |
| `src/common/point_lio_ros2` | [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) | `a8e2d0d` | `patches/point_lio_ros2.patch` (ZUPT、`use_sim_time` 引数、RViz の設定) |
| `src/l1/unilidar_sdk` | [unitreerobotics/unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk) | `1bd7d95` (v1.0.16) | なし |
| `src/l2/unilidar_sdk2` | [unitreerobotics/unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2) | `0e3c51f` (v2.0.10) | `patches/unilidar_sdk2.patch` (ドライバの launch に接続方式の引数) |
| `src/l2/lidar_localization_ros2` | [rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) | `cd5e27a` | なし |
| `src/l2/ndt_omp_ros2` | [rsasaki0109/ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) | `25f3489` | なし |

```bash
./scripts/fetch-external.sh          # 取得とパッチ (setup-raspi.sh が自動で実行する。何度実行してもよい)
./scripts/fetch-external.sh l1       # common と l1 だけ
```

* 使わないファイル (Point-LIO の README 用の画像 約 250 MB、Unilidar SDK の ROS1 版や docs) は取りません
  (部分クローンと sparse-checkout)。`git submodule update --init` で全部取った場合も、
  ROS1 版には `COLCON_IGNORE` を置き、Docker には `.dockerignore` で入れません。
* 外部リポジトリの中身を変えたいときは、変更をパッチにして `patches/<ディレクトリ名>.patch` に置きます
  (`git -C src/common/point_lio_ros2 diff > patches/point_lio_ros2.patch`)。
* 取り込み元のコミットを変えるときは、サブモジュールの中でそのコミットに切り替えてから
  `git add src/common/point_lio_ros2` でコミットし、パッチが当たるか `fetch-external.sh` で確かめます。
* 以前このリポジトリに取り込んでいたころのクローンで `git pull` すると、消えずに残ったファイル (地図など) は
  `fetch-external.sh` が `.external-backup/` に退避してから取得します。

## セットアップ (Ubuntu 22.04 / Raspberry Pi)

Raspberry Pi (Ubuntu 22.04 / arm64) や PC に ROS 2 Humble と本リポジトリを入れます。
`scripts/setup-raspi.sh` が ROS 2 のインストール、依存パッケージ、`dialout` グループへの追加、
外部リポジトリの取得、ビルド、`~/.bashrc` への設定追記までを行います (何度実行してもよい)。

```bash
git clone https://github.com/0xAAA8000/Docker-Unitree-ROS2.git ~/Docker-Unitree-ROS2
cd ~/Docker-Unitree-ROS2
./scripts/setup-raspi.sh l1          # L2 なら l2。l1 と l2 は同じワークスペースに入れられない
# メモリ不足で Point-LIO のビルドが落ちるときは JOBS=1 ./scripts/setup-raspi.sh l1
source ~/.bashrc
```

* ビルド先は `~/ros2_ws` (`WS=` で変更可)。`--symlink-install` なので、リポジトリの yaml / launch /
  Python を書き換えれば再ビルドなしで反映されます。C++ を変えたら同じスクリプトを再実行します。
* 新しいパッケージを足したとき、C++ を変えたとき、外部リポジトリのコミットやパッチを変えたときも、このスクリプトを再実行します。
* ふだんはスマホのモード選択 ([下記](#スマホでモードを選んで起動する-robot_launcher)) から起動し、
  Arduino を挿したら自動で起動するようにできます ([下記](#arduino-を挿したら自動で起動する-install-autostartsh))。
* rviz2 は入れていません。同じネットワークの PC で `ROS_DOMAIN_ID` をそろえて rviz2 を開きます
  (ラズパイにも入れるなら `WITH_RVIZ=1`)。
* ラズパイには RTC が無いので、起動直後は時計がずれていて、NTP で同期した瞬間に時刻が飛びます。
  Point-LIO は時刻が戻ると `loop back` でデータを捨てるので、`timedatectl` で
  `System clock synchronized: yes` になってから起動してください。
* L1 (`/dev/ttyUSB*`) と Arduino (`/dev/ttyACM*`) は挿す順で番号が変わることがあるので、
  `robots/<名前>.yaml` の `robot.lidar.port` / `robot.arduino_port` を `/dev/serial/by-id/...` にするのが確実です。

## 設定 (`robot_config`)

全ノードの設定を `src/common/robot_config/config/` にまとめてあります。各ノードの launch が、起動するときにここから値を配ります
(ノードごとの設定ファイルはありません)。yaml を書き換えたら、ビルドし直さずに次の起動から反映されます。

| ファイル | 中身 |
| --- | --- |
| `common.yaml` | ロボットに依らない設定。`nodes.<ノード>` にノードごとのパラメータ (Nav2 は `nodes.nav2`) |
| `robots/rc_car.yaml` | 実機。`robot:` にロボットの事実 (LiDAR の機種・ポート・取付、車体の footprint・地面の高さ・自車体の箱・ホイールベース、Arduino のポート)、`nodes:` にこのロボットだけの値 |
| `robots/sim.yaml` | Gazebo のシミュレーション |

* 各ノードに渡る値は、`common.yaml` の値に、ロボットの事実から計算した値 (自車体の箱、Nav2 の footprint、
  後輪軸から LiDAR までの距離など) と `robots/<名前>.yaml` の `nodes:` を重ねたものです (後のものが優先)。
* 使うロボットは launch 引数 `robot:=<名前>`、なければ環境変数 `ROBOT_NAME`、なければ `rc_car` です。
  新しいロボットは `robots/` にファイルを足します。
* キャリブレーションの結果 (`~/drive_calib/.../summary.yaml`) は含めません。モード選択の画面で選ぶか `calibration_file:=` で渡します。
* launch 引数の `lidar:=` / `lidar_port:=` / `arduino_port:=` / `web_port:=` で、その起動のときだけ設定を上書きできます。
* LiDAR の機種ごとの Point-LIO の設定 (`point_lio_ros2/config/unilidar_<l1|l2>.yaml`) は外部パッケージの設定なので
  そのまま土台にし、`common.yaml` の `nodes.point_lio` を上に重ねます。Gazebo のモデル (URDF など) と L2 の地図作り用の
  `l2_bringup` の設定は対象外です。
* 実際に渡る値は次のコマンドで確認できます。

```bash
ros2 run robot_config show                          # ロボットの事実とノードの一覧
ros2 run robot_config show obstacle_detector        # そのノードに渡る値
ros2 run robot_config show nav2 --robot sim         # sim の Nav2 のパラメータ
```

## スマホでモードを選んで起動する (`robot_launcher`)

スマホで `http://<IP>:8080` を開くと、モード選択の画面が出ます。モードを選ぶと、進み具合の円と
待っている段階 (前のモードの停止 → LiDAR の点群 → 自己位置 → Nav2 → モードの画面) が表示され、
準備ができたらモードの画面 (`http://<IP>:8081`) へ自動で移ります。モードの画面の「モード」ボタンで戻れます。

| モード | 起動するもの |
| --- | --- |
| 目的地設定の自走 | `robot.launch.py` 一式 (LiDAR、Point-LIO + ZUPT、地図、Nav2、Arduino 変換、目的地の画面) |
| キャリブレーション | LiDAR と Point-LIO (ZUPT なし) と、キャリブレーションの画面 (`calib_web`)。Nav2 と Arduino 変換は起動しない |
| マップ自動生成の自走 | (準備中。探索ノードが未実装) |

* キャリブレーションの画面では、ホイールベースと後輪軸から LiDAR までの距離を入れ、「周りに 3 m の空きがある」に
  チェックしてから計測を始めます。`calibrate_drive` の出力が画面に流れ、中止ボタンで止められます。
  計測中に画面との通信が 5 秒途切れると、計測は中止されます。
* 計測結果 (`~/drive_calib/<日時>/summary.yaml`) は画面に一覧され、「使う」で選んだものが
  `~/drive_calib/selected` に記録されます。次に「目的地設定の自走」を起動するときに `calibration_file:=` として渡ります。
* モードは同時に 1 つだけ動きます (Arduino のポートの取り合いを防ぐ)。問題の切り分けのため、
  LiDAR ドライバや Point-LIO も含めて、モードを選ぶたびに起動し直します (切り替えに 10〜20 秒ほど)。
  起動し直すので、地図と自己位置はモードごとに最初からになります。
* モードごとの出力は `~/robot_logs/<日時>_<モード>.log` に保存されます。起動に失敗したときは、
  止まった段階とログの最後の部分が画面に出ます。
* 目的地設定の自走では、`robot_map` が貯めた地図 (ボクセル地図) を `~/robot_maps/<日時>_goal.pcd` に保存します
  (60 秒ごとと、モードを止めたとき)。キャリブレーションのモードは地図を作りません。
* 自動起動 (`scripts/start_ros.sh`、`scripts/autostart/robot-bringup.sh`) はこの `launcher` を起動します。
  モードを順に止める時間が要るので、サービスの `TimeoutStopSec` は 60 秒程度にしてください。
* `launcher` のポートや保存先は `robot_config` の `nodes.launcher` から読みます。各モードにはロボット名 (`robot:=`) を渡します。

```bash
ros2 run robot_launcher launcher                                   # 手で起動する場合
ros2 run robot_launcher launcher --ros-args -p token:=abcd -p launch_args:="dry_run:=true"
```

## スマホから地図をタップして目的地を送る (`robot_map` / `robot_goal web`)

走りながら 2D 地図を作り、同じ Wi-Fi につないだスマホのブラウザでその地図をタップして目的地を送ります。
2D 地図を作るのは `robot_map` だけで、Web 画面、RViz (`Map2D` 表示)、必要なら Nav2 がその `/map` を共有します。

![LiDAR から地図・Web・走行までのデータの流れ](docs/images/data_flow.svg)

線の色はデータの種類 (橙: 3D 点群、青: 2D 地図・経路、灰: IMU・オドメトリ・指令、緑: HTTP)、
破線は任意またはファイル経由です。トピック名は `real.launch.py` で起動したときの実際の名前です。

### 起動

ふだんはモード選択の「目的地設定の自走」で起動し、`http://<IP>:8081` に自動で移ります。
`robot.launch.py` を直接起動したときは `http://<IP>:8080` です (既定で `map:=true web:=true`。起動ログに
`スマホで開く: http://192.168.x.x:8080` と出ます)。

```bash
ros2 launch robot_bringup robot.launch.py                        # 地図作り + Web
ros2 launch robot_bringup robot.launch.py web_token:=abcd        # http://<IP>:8080/?key=abcd の人だけ操作できる
ros2 launch robot_bringup robot.launch.py map_pcd:=$HOME/robot_maps/<日時>_goal.pcd   # 保存済みの地図から始める
```

単体で起動する場合:

```bash
ros2 launch robot_map map.launch.py                  # /cloud_registered -> /map
ros2 run robot_goal web                              # /map を表示, ポート 8080
ros2 run robot_goal web --ros-args -p port:=8082 -p token:=abcd
```

* Web サーバーはノードを動かしているマシンで開きます。Point-LIO と同じマシン
  (ラズパイなど) で動かすと、点群を Wi-Fi 越しに送らずに済みます。
* `ufw` などのファイアウォールを使っている場合はポートを開けます: `sudo ufw allow 8080/tcp`
* スマホがインターネットにつながっていなくても動きます (画面は全部このノードから配信)。
* `web_token` を付けないと、同じ Wi-Fi にいる人なら誰でもロボットを動かせます。

### 画面の操作

| 操作 | 内容 |
| --- | --- |
| 地図をタップ | 目的地の候補 (橙) を置く。向きは今の向きのまま |
| 候補の印からドラッグ | 到着時の向きを指定する |
| 1 本指でドラッグ / 2 本指で広げる・縮める | 地図の移動 / 拡大・縮小 (移動すると「追従」が切れる) |
| この地点へ送信 | 候補を Nav2 に送る (送るまでは走り出さない) |
| 停止 (取消) | 走行中のゴールを取り消す |
| 地図 / 全体コスト / 近くコスト / 3D | 表示の切り替え ([下記](#表示の切り替え-2d-の-costmap-と-3d-地図)) |
| 追従 / 全体 | ロボットを中心に表示 / 観測済みの範囲を全部表示 (3D では視点を車両の斜め後ろに戻す) |
| 障害物を消す | Nav2 の costmap に残った障害物 (誤検知) をすべて消す。見えている障害物はすぐ付き直る |
| モード | モード選択の画面に戻る (モード選択から起動したときだけ出る) |

上の帯に、地図・現在地の受信状況、Nav2 の接続、走行状態と残り距離が出ます。

### 表示の切り替え (2D の costmap と 3D 地図)

| 表示 | 中身 |
| --- | --- |
| 地図 | `robot_map` の 2D 地図 (`/map`) |
| 全体コスト | Nav2 の経路計画用の costmap (`/global_costmap/costmap`、車体の周り 30 m) |
| 近くコスト | Nav2 の経路追従用の costmap (`/local_costmap/costmap`、車体の周り 3 m) |
| 3D | `robot_map` の 3D 地図 (ボクセル) のうち、車両から見えている範囲 |

* costmap の色: 黒が障害物、赤がロボットの中心が入るとぶつかる範囲、オレンジが障害物から離れたい範囲 (濃いほど近い)、
  灰色が未知です。Nav2 には毎回全体を配信させています (`always_send_full_costmap: true`)。
* 3D: 形は Point-LIO の点群から作ったボクセル (Point-LIO の地図の粗さ 0.1 m)、色は `obstacle_detector` の
  分類 (床・坂・障害物。障害物は 2 回見てから付ける。車体より高い所は未分類) です。範囲は、直近 1 秒の点が
  届いた水平距離の 95% 点 (上限 15 m) の円の中です。1 本指で車両を中心に回転、2 本指で拡大・移動します
  (2 本指で動かすと「追従」が切れます)。
* 取りに行くのは表示中のものだけです。3D は最初に範囲内の全体を、そのあとは 1 秒ごとに差分 (増えた・
  色が変わった・範囲から出たボクセル) だけを受け取ります。3D 表示の Three.js (約 670 KB) は最初に 3D を開いたときに
  読み込みます (ロボットから配信するので、スマホがインターネットにつながっていなくても使えます)。

### 地図の作り方 (`robot_map`)

Point-LIO の `/cloud_registered` (map 座標の点群、毎スキャン) を base_link 座標に直して高さで分け、
5 cm 四方のセルごとに数えます。

* 床から ±`floor_tolerance` (5 cm) の点 → 床、`obstacle_min_height`〜`obstacle_max_height` の点 → 障害物。
  `ground_z` と障害物の高さは `obstacle_detector` と同じ値 (`robot_config` から) を使います。
* 障害物の点が 3 回以上で、かつ床の点の半分以上ならそのセルは占有、床だけなら空き、どちらも無ければ未知です。
  人が横切った跡は、その後に床がたくさん見えれば空きに戻ります。
* ロボットから `max_range` (15 m) より遠い点は使いません。
* `/map` は変化があったときに 1 秒ごとに配信します (transient_local なので、後から起動したノードも受け取れます)。
* `pcd_file` を指定すると、保存済みの地図 (下のボクセル地図や、Point-LIO の `scans.pcd`) を最初に読み込みます。
  床の高さは点の高さの分布から自動で推定します (`pcd_ground_z` で指定も可)。
* **地図の保存 (ボクセル地図)**: 同じ点群を、Point-LIO の地図の粗さ (`filter_size_map`、0.1 m) のボクセルにまとめて貯め、
  60 秒ごとと終了時に `.pcd` (ボクセルの中心) に書きます。保存先は `save_file`
  (`robot.launch.py` の `map_save:=`、空なら `~/robot_maps/<起動日時>_map.pcd`)。2 回以上のスキャンで点が入った
  ボクセルだけを保存します (通った人などを除く)。同じ場所を何度見ても 1 つにまとまるので、100 万ボクセルで約 10 MB です。
* Point-LIO の地図の保存 (`pcd_save_en`) は止めてあります。全スキャンの点をまとめずにメモリに貯め続け、
  10 分ほどで数百 MB になるためです。
* Nav2 は今のところ `/map` を使いません (global costmap は rolling window のまま)。図の Nav2 への `/map` は将来の案です。


## コマンドで起動する (`robot_bringup` / `robot_goal`)

モード選択を使わずに、ターミナルから起動する方法です。LiDAR ドライバ、lidar_bridge + 障害物の検知 + Point-LIO、
Nav2、Arduino 変換、2D 地図 (`robot_map`)、スマホ用の Web サーバー (`robot_goal web`) を 1 つのコマンドで起動します。

```bash
ros2 launch robot_bringup robot.launch.py                       # ロボットは $ROBOT_NAME (無ければ rc_car)
ros2 launch robot_bringup robot.launch.py calibration_file:=$HOME/drive_calib/<日時>/summary.yaml
ros2 launch robot_bringup robot.launch.py lidar_port:=/dev/ttyUSB1   # 設定の一時的な上書き (lidar / lidar_port / arduino_port / web_port)
ros2 launch robot_bringup robot.launch.py dry_run:=true         # Arduino に送らずログだけ
ros2 launch robot_bringup robot.launch.py nav:=false operation:=false   # 地図作り・確認だけ
```

* `driver` / `point_lio` / `nav` / `operation` / `map` / `web` / `rviz` を `false` にすると
  その部分を起動しません (`rviz` は既定で `false`。PC 側で開く)。
* Nav2 は Point-LIO の TF が出始めるのを待つため `bringup.nav_delay` 秒 (既定 5) 後に起動します。

目的地は別のターミナルから送ります (座標は `map`、Point-LIO の起動位置が原点)。

```bash
ros2 run robot_goal goal 3.0 1.5        # (3.0, 1.5) へ。向きは今のまま
ros2 run robot_goal goal 3.0 1.5 90     # 到着時の向き [deg]
ros2 run robot_goal goal --rel 2.0 0    # 今の位置・向きから見て前に 2 m
ros2 run robot_goal goal                # 対話モード (h でヘルプ, c で取り消し, p で現在地)
```

到着・失敗まで残り距離を表示し、Ctrl-C でゴールを取り消して終了します。

### Arduino を挿したら自動で起動する (`install-autostart.sh`)

Arduino (`/dev/ttyACM0`) が現れたらモード選択の画面 (`robot_launcher` の `launcher`) を起動し、抜いたら止めます。
udev が systemd のサービス `robot-bringup.service` を起動する仕組みです。

```bash
./scripts/install-autostart.sh l1          # sudo は付けない (ロボットを動かすユーザーで実行). L2 なら l2
./scripts/install-autostart.sh uninstall   # 元に戻す
```

* 設定は `/etc/default/robot-bringup` に書きます (ワークスペースの場所、`ROS_DOMAIN_ID`、使うロボット `ROBOT_NAME`、
  launch 引数 `ROBOT_LAUNCH_ARGS="calibration_file:=... web_token:=..."` など)。ポートなどロボットの設定は
  `robot_config` の `robots/<名前>.yaml` に書きます (`ARDUINO_PORT` は udev が監視するデバイスだけに使う)。
  変更したら `sudo systemctl restart robot-bringup` するか、Arduino を挿し直します。
* ログは `journalctl -u robot-bringup -f`、状態は `systemctl status robot-bringup` で見ます。
* systemd から起動したプロセスは `~/.bashrc` を読みません。そのため
  `scripts/autostart/robot-bringup.sh` が ROS とワークスペースの `setup.bash` を読み込んでから起動します。
* 止めるときは `launcher` に停止の合図が届き、動いているモードを順に止めてから終わります
  (最大 30 秒ほど。サービスの `TimeoutStopSec` は 60 秒)。
* 起動に失敗したら 10 秒後にやり直します (LiDAR やネットワークの準備がまだのとき)。

## LiDAR から Arduino までの仕組みと調整

sim と同じトピック・TF にそろえて、Point-LIO → Nav2 → Arduino の順につなぎます。
ふだんは上の一括起動やモード選択で起動しますが、1 つずつ起動することもできます。

| ノード | 受け取る | 出す |
| --- | --- | --- |
| unitree_lidar_ros2 (LiDAR ドライバ) | — | `/unilidar/cloud`, `/unilidar/imu` |
| lidar_bridge (名前の付け替え) | `/unilidar/cloud`, `/unilidar/imu` | `/lidar`, `/imu` (Point-LIO 用), `/lidar_full` (障害物の検知用) |
| Point-LIO | `/lidar`, `/imu` | 自己位置 |
| obstacle_detector (障害物の検知) | `/lidar_full` | `/lidar_obstacles`, `/lidar_clear` (Nav2 用), `/lidar_slope` (確認用) |
| Nav2 | 自己位置, `/lidar_obstacles`, `/lidar_clear` | `/cmd_vel` |
| cmd_vel_converter | `/cmd_vel` | シリアル (Arduino) |

```bash
# 1. LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py
# 2. ブリッジ + 障害物の検知 + 静的 TF + Point-LIO (機種はロボットの設定の lidar.model)
ros2 launch lidar_bridge real.launch.py
# 3. Nav2 (Gazebo で動かすときは robot:=sim)
ros2 launch robot_navigation navigation.launch.py
# 4. Twist -> Arduino (最初は dry_run:=true で指令値だけ確認するとよい)
ros2 launch robot_operation operation.launch.py calibration_file:=$HOME/drive_calib/<日時>/summary.yaml
```

* 設定はすべて `robot_config` にあります ([下記](#設定-robot_config))。Nav2 のパラメータは共通部分が `common.yaml` の
  `nodes.nav2`、実機 (前後 0.50 m x 横幅 0.30 m の車体、幅 0.80 m のドアを通れる設定) の分が `robots/rc_car.yaml`、
  sim の分が `robots/sim.yaml` です。`use_sim_time`、`/cmd_vel` の行き先、footprint もロボットの設定から決まります。
* LiDAR の取付位置・傾き (前に 30 度 → `pitch_deg: 30`)、地面の高さ、自車体の箱は `robots/<名前>.yaml` の
  `robot.lidar.mount` と `robot.body` で設定します。障害物の高さ判定は、この取付姿勢で
  base_link 座標に直してから行います (配信する点の座標は LiDAR 座標のまま)。
* 経路計画用の costmap (global) は、LiDAR の光線が通っていない場所 (壁の裏など) を「未知」として扱い、
  経路計画はそこを一番重いコストで扱います (通れなくはないが、見えている道の約 4 倍のコスト)。
  経路追従用 (local) は、未知を衝突とみなして止まらないよう、未知を空きとして扱います。
* 止まっている間は Point-LIO に「速度 = 0」を観測として入れ (ZUPT)、IMU の加速度のずれで位置が流れるのを防ぎます。
  `stationary_detector` が、速度指令 (`/cmd_vel`) が 1 秒以上止まっていて、かつ IMU が静かなときに
  `/stationary` を true にします (`real.launch.py` が起動)。**速度指令を出さずに動かすとき**
  (手で持って動かす試験、`calibrate_drive` で Arduino を直接動かすときなど) は、
  等速で動いていると止まっていると誤判定するので `zupt:=false` で起動してください。
* 障害物 (`/lidar_obstacles`) は LiDAR から `obstacle_range_min` (0.08 m) 以上の点から作ります
  (Point-LIO は自分の `preprocess.blind` (0.5 m) より近い点を捨てるので、それとは別。車体の点は `self_box_*` で除きます)。
  Nav2 のクリアには `/lidar_clear` を使います。床の点を LiDAR から水平に `ground_clear_extend` (0.10 m) 奥へずらしてあり、
  床が見えているセルに残った誤検知の障害物も消えます。それでも残るときは、目的地の画面の「障害物を消す」で
  costmap の障害物をすべて消せます (見えている障害物はすぐ付き直ります)。
* LiDAR は前に傾けて付けているので、車体の真横の低い所と後ろの低い所は見えません。
* 坂: 高さで障害物になった点のうち、床から傾き `slope_max_deg` (15°) 以下・隣との段差 `step_max` (0.06 m) 以下で
  つながっている所は坂として障害物から外します (`slope_filter`。車体の周り ±4 m)。箱や台は上面が平らでも、
  縁の段差で床からつながらないので障害物のままです。点がまばらで途切れた坂は障害物のまま (安全側) です。
  外した点は `/lidar_slope` に出るので rviz で確認できます。傾きは車体に対する値です。
* 床が障害物として扱われる (Nav2 の経路が作れない) ときは、取付角度か `ground_z` が実機とずれています。
  平らな床に止めて LiDAR ドライバだけ起動し、`ros2 run lidar_bridge ground_check` を実行すると、
  床の点から取付の `pitch_deg` / `roll_deg` と地面の高さを推定し、`robots/<名前>.yaml` の今の値
  (`robot.lidar.mount`、`robot.body.ground_z`) と並べて表示します。
  ロボットから 1 m 以内に障害物の点があれば (車体や取付台が写っている)、
  それを含む `robot.body.self_box_min` / `self_box_max` の例も表示します。
* `cmd_vel_converter` は Twist を `tan(δ) = L·ω / v` で舵角に、`sin(δ) = K·sin(φ − φ0)` でサーボ角に、
  `throttle = deadband + v / gain` でスロットルに変換します。値は `calibration_file`
  (calibrate_drive の `summary.yaml`) か、ロボットの設定 (`robots/<名前>.yaml` の `nodes.cmd_vel_converter`) から読みます。
* キャリブレーション後に遅い指令でモーターが止まってしまう (トルク不足) ときは、`robots/<名前>.yaml` の
  `nodes.cmd_vel_converter` の
  `min_forward_throttle` / `min_backward_throttle` に「止まった状態から確実に動き出すスロットル」を設定します。
  動く間はこの値を下回りません (キャリブレーションの結果では上書きされません。0 で無効)。
* `/cmd_vel` が 0.5 秒途絶えるとスロットル 0 を送ります。
* LiDAR (L1 は `/dev/ttyUSB0`) と Arduino の USB シリアルが同じ名前にならないよう、
  `robots/<名前>.yaml` の `robot.lidar.port` / `robot.arduino_port` を `/dev/serial/by-id/...` にするのが確実です。

### 決めた秒数だけ前進して点群を記録する (`drive_forward`)

上の 1, 2, 4 を起動した状態 (Nav2 は不要) で実行します。`/cmd_vel` に一定速度を送り、
`--bag` を付けると走り出す前から止まった後までの生データと Point-LIO の出力を記録します。

```bash
ros2 run robot_operation drive_forward --speed 0.3 --duration 5 --bag ~/bags/forward1
```

* Docker のコンテナで動かすときは、記録先をコンテナを終了しても消えない、マウントしている場所 (`/opt/common_ws/src` など) にします。
* Ctrl-C で中止すると速度 0 を送ってから終了します。
* 地図は `robot_map` が保存します ([地図の作り方](#地図の作り方-robot_map))。`real.launch.py` だけを起動したときは保存されません
  (Point-LIO の保存は止めてある)。

## 走行キャリブレーション (`robot_operation calibrate_drive`)

実機を自動で走らせて、次の2つの対応を Point-LIO のオドメトリから計測します。

* スロットル (operator.ino の -100..100) と速度 [m/s]
* サーボ角 [deg] と前輪の舵角 [deg]

ふだんはスマホのモード選択で「キャリブレーション」を選んで計測します ([上記](#スマホでモードを選んで起動する-robot_launcher))。
ホイールベースと後輪軸から LiDAR までの距離の初期値は、ロボットの設定から計算した値が画面に入ります。
ターミナルから計測する場合:

```bash
# ターミナル1: LiDAR ドライバと Point-LIO だけ (ZUPT なし。車体は静止させておく)
ros2 launch robot_bringup robot.launch.py nav:=false operation:=false map:=false web:=false zupt:=false

# ターミナル2: 計測 (半径 3 m ほどの空きが必要). 値は ros2 run robot_config show calib_web で確認できる
ros2 run robot_operation calibrate_drive --wheelbase <ホイールベース[m]> --lidar-x <後輪軸からLiDARまでの前方向距離[m]>
```

* 手順は2段階です。まずステアリングを中央にして各スロットルで直進し、同じだけ後退して戻ります。次に各サーボ角で、一定のスロットルのまま1周（または時間切れ）するまで旋回します。
* LiDAR は前に 30 度傾いていても、X 軸が前を向いていれば問題ありません。機体 X 軸を水平面に投影した向きをヨー角として使います。
* 舵角は自転車モデル `tan(δ) = L / R` で求めます。旋回半径 R は、LiDAR の位置から後輪軸中心の値に換算します (`--lidar-x`)。
* 結果は `~/drive_calib/<日時>/` に保存されます (`robot_config` には含めず、起動時に `calibration_file:=` で渡します)。生データ `raw.csv`, `segments.csv`、区間ごとの結果 `throttle_result.csv`, `steering_result.csv`、フィット結果 `summary.yaml` の5つです。
* `--analyze <ディレクトリ>` を付けると走行せず、記録済みのデータを別の `--wheelbase` / `--lidar-x` で解析し直せます。
* Ctrl-C、またはオドメトリが 0.5 秒途絶えた場合は、スロットル 0・サーボ中央を送って止めます。ただし Arduino 側にはタイムアウトがないので、USB が抜けたときに備えてすぐ止められる状態で使ってください。
* 主なオプション: `--throttles 15,20,30`, `--servos 40,55,70,85,100,115,130`, `--steer-throttle 25`, `--max-distance 3.0`, `--pause` (区間ごとに Enter 待ち)。一覧は `--help` で確認できます。

## Point-LIO だけを動かす

実機 (Unitree L2) の場合:

```bash
# ターミナル1: LiDAR ドライバ
ros2 launch unitree_lidar_ros2 launch.py

# ターミナル2: Point-LIO
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

L1 の場合は `mapping_unilidar_l1.launch.py` を使ってください。
`avia` / `horizon` / `mid360` / `ouster64` / `velody16` 用の launch と、
オドメトリのみを出す `correct_odom_unilidar_l1(l2).launch.py` も同梱しています。

パッケージ名は `point_lio`、実行ファイルは `pointlio_mapping` です
(ディレクトリ名の `point_lio_ros2` とは異なります)。

### launch 引数

| 引数 | 既定値 | 説明 |
| --- | --- | --- |
| `rviz` | `true` | RViz2 を同時に起動する |
| `use_sim_time` | `false` | bag 再生や Gazebo で回すときは `true` (unilidar 用 launch のみ、下記「Point-LIO のバージョンについて」参照) |

### トピック

* 購読: `common.lid_topic` (既定 `/unilidar/cloud`), `common.imu_topic` (既定 `/unilidar/imu`)
* 配信: `/pointlio/odom`, `/pointlio/path`, `/pointlio/cloud_registered`,
  `/pointlio/cloud_registered_body`, `/pointlio/laser_map`
* TF: `camera_init` → `aft_mapped`
  (`odom_header_frame_id` / `odom_child_frame_id` パラメータで変更可能)

パラメータは `src/common/point_lio_ros2/config/*.yaml` (取り込み元のまま) にあります。`real.launch.py` で起動したときは、
これに `robot_config` の `nodes.point_lio` を重ねます。この節の `mapping_unilidar_*.launch.py` で起動すると
`robot_config` の値 (`fov_degree: 360` や `init_map_size: 1000` など) は入らないので、実機の動きを確かめるときは `real.launch.py` を使います。

動作確認や、うまく動かないときの切り分けは
[ROS 2 コマンド チートシート](docs/ros2-cheatsheet.md#14-トラブルシュート早見表)
を参照してください。


## Point-LIO のバージョンについて

Unitree 公式の [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)
は catkin / roscpp ベースの **ROS 1 専用パッケージ**で、Humble ではビルドできません。

そこで、ROS 2 に移植済みの [**dfloreaa/point_lio_ros2**](https://github.com/dfloreaa/point_lio_ros2) を
サブモジュール (`src/common/point_lio_ros2/`、コミット `a8e2d0d`、2025-08-14) として使っています。
これは hku-mars の Point-LIO を ROS 2 に移植したうえで、Unitree Unilidar L1 / L2 対応
(上流 `point_lio_unilidar` 由来) を取り込んだものです。

`patches/point_lio_ros2.patch` で次の変更を当てています。

1. ZUPT (止まっている間は速度 0 を観測として入れる。`zupt.enable` / `zupt.topic` / `zupt.vel_cov` / `zupt.timeout`)
2. `mapping_unilidar_l1.launch.py` / `mapping_unilidar_l2.launch.py` に `use_sim_time` 引数 (Gazebo / bag 再生用)
3. RViz の表示設定

設定 (`config/unilidar_<l1|l2>.yaml`) は取り込み元のままで、こちらで変えた値は `robot_config` に置いています
(`common.yaml` の `nodes.point_lio` と、実機の `robots/rc_car.yaml` の `fov_degree: 360`・`publish_odometry_without_downsample: false` など)。
`real.launch.py` で起動するとこれらが重なります。

上流の `image/` (README 用のデモ GIF、約 246 MB) と、実体のないサブモジュール定義の `.gitmodules` は取りません
(`include/IKFoM` と `include/ikd-Tree` は上流でも実ファイルとしてコミット済み)。

Livox AVIA の `CustomMsg` 入力経路は上流でもコメントアウトされており
(`livox_ros_driver2` を含めていないため)、標準の `sensor_msgs/PointCloud2` 経路のみが有効です。

## シミュレーション (Gazebo)

`src/sim/sim_gazebo` に Gazebo (Ignition) の車体とワールドがあります。設定は `robot_config` の `robots/sim.yaml` を使います
(Docker の sim イメージでは `ROBOT_NAME=sim` になっています)。

```bash
ros2 launch sim_gazebo bridge.launch.py                       # Gazebo、ブリッジ、点群の変換、障害物の検知、車体
ros2 launch robot_navigation navigation.launch.py robot:=sim  # Nav2 (sim の速度・footprint、Gazebo の時刻)
```

* `filter_invalid_points` が Gazebo の点群から無効な点と自車体の点を除き、Point-LIO 用の time を付けて `/lidar` に出します。
  障害物の判定は実機と同じ `obstacle_detector` です。
* Point-LIO は `/lidar` と `/imu` を、Gazebo の時刻で読ませて起動します
  (例 `ros2 run point_lio pointlio_mapping --ros-args --params-file <unilidar_l2.yaml> -p common.lid_topic:=/lidar -p common.imu_topic:=/imu -p use_sim_time:=true`)。
* ワールドの作り方は `src/sim/sim_gazebo/readme.md` にあります。

## Docker で動かす (任意)

Docker イメージは L1 / L2 / sim (Gazebo) の 3 種類です。ビルドの前に外部リポジトリを取得しておきます
(イメージには `src/` の中身がそのまま入るため。GitHub Actions のビルドでは自動で実行します)。

sim のイメージでは環境変数 `ROBOT_NAME=sim` が入っているので、`robot:=` を付けなくても sim の設定を使います。

```bash
./scripts/fetch-external.sh
docker pull ghcr.io/0xaaa8000/docker-unitree-ros2/l1:latest     # l2, sim も同様
./scripts/launch-docker.sh --target=sim --dev                   # --dev で src/{l1, l2, sim} をマウント
```

## Licenses & Acknowledgments

This project utilizes and depends on the following open-source software:

* **ROS 2 (desktop-full)**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) (Some individual packages within the distribution may use BSD or other permissive licenses)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc. and ROS 2 contributors
  * **Source:** [ROS 2 Documentation - Governance and Policies](https://docs.ros.org/en/humble/Governance/ROS2-Open-Source-Policy.html)

* **Unitree_Lidar_ROS2**
  * **License:** [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)
  * **Copyright:** Copyright (c) Unitree Robotics
  * **Source:** [unitreerobotics/unitree_lidar_ros2 (GitHub)](https://github.com/unitreerobotics/unitree_lidar_ros2)

* **Point-LIO / point_lio_ros2**
  * **License:** [GNU General Public License v2.0](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html)
    (`src/common/point_lio_ros2/LICENSE`)。上流の `package.xml` には `BSD` と書かれていますが、
    同梱の `LICENSE` は GPL-2.0 です。**Docker イメージなど、Point-LIO を含めて配布するものには
    GPL-2.0 の条件がかかる点に注意してください。**
  * **Copyright:** Copyright (c) Dongjiao He, Wei Xu (HKU MARS Lab) / Daniel Florea (ROS 2 port & Unilidar support)
  * **Source:** [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO),
    [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2),
    [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar)

* **Unilidar SDK / SDK2** (`src/l1/unilidar_sdk`, `src/l2/unilidar_sdk2`)
  * **License:** [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause) (同梱の `LICENSE`。`package.xml` には GPLv3 と書かれています)
  * **Source:** [unitreerobotics/unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk),
    [unitreerobotics/unilidar_sdk2](https://github.com/unitreerobotics/unilidar_sdk2)

* **lidar_localization_ros2 / ndt_omp_ros2** (`src/l2/`)
  * **License:** [BSD 2-Clause License](https://opensource.org/licenses/BSD-2-Clause)
  * **Source:** [rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2),
    [rsasaki0109/ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2)

* **Gazebo Classic / gazebo_ros_pkgs**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc.
  * **Source:** [ros-simulation/gazebo_ros_pkgs (GitHub)](https://github.com/ros-simulation/gazebo_ros_pkgs)
