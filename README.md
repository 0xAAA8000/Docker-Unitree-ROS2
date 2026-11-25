# Unitree LiDAR 自走ロボット (ROS 2 Humble)

Unitree L1 / L2 LiDAR と Point-LIO で自己位置を求め、Nav2 で経路を計画し、Arduino でアッカーマン (前輪操舵) の
RC カーを走らせるためのリポジトリです。スマホのブラウザからモードを選び、地図をタップして目的地を送れます。

> ROS 2 のコマンドの使い方は **[ROS 2 コマンド チートシート](docs/ros2-cheatsheet.md)** にまとめてあります。

もとは Docker イメージ用のリポジトリでしたが、今はロボットに直接入れて使う形を基本にしています。
Docker イメージ (sim 含む) は[最後の節](#docker-で動かす-任意)で引き続き作れます。

## 中身

| 場所 | 内容 |
| --- | --- |
| `src/common/` | 全機種共通のパッケージ (下の表) |
| `src/l1/`, `src/l2/` | LiDAR の機種ごとのパッケージ (ドライバなど) |
| `src/sim/` | Gazebo のシミュレーション (`sim_gazebo`) |
| `firmware/` | Arduino のスケッチ (`operator.ino`) |
| `scripts/` | セットアップ (`setup-raspi.sh`)、外部リポジトリの取得 (`fetch-external.sh`)、自動起動など |
| `patches/` | 外部リポジトリに当てる、こちらの変更 |

### このリポジトリのパッケージ (`src/common/`)

| パッケージ | 内容 |
| --- | --- |
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
| `src/common/point_lio_ros2` | [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) | `a8e2d0d` | `patches/point_lio_ros2.patch` (ZUPT、L1 の設定、`use_sim_time` 引数、RViz の設定) |
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
  `/dev/serial/by-id/...` で指定するのが確実です。

## 実機で Nav2 を動かす (`lidar_bridge` / `robot_operation`)

sim と同じトピック・TF にそろえて、Point-LIO → Nav2 → Arduino の順につなぎます。

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
# 2. ブリッジ + 静的 TF + Point-LIO (L1 なら lidar:=l1)
ros2 launch lidar_bridge real.launch.py lidar:=l2
# 3. Nav2 (既定が実機用. Gazebo で動かすときは sim:=true)
ros2 launch robot_navigation navigation.launch.py
# 4. Twist -> Arduino (最初は dry_run:=true で指令値だけ確認するとよい)
ros2 launch robot_operation operation.launch.py calibration_file:=$HOME/drive_calib/<日時>/summary.yaml
```

* Nav2 のパラメータは実機用が `src/common/robot_navigation/config/nav2_params_real.yaml`
  (前後 0.50 m x 横幅 0.30 m の車体、幅 0.80 m のドアを通れる設定)、sim 用が `nav2_params.yaml` です。
  `navigation.launch.py` は既定で実機用 (`use_sim_time:=false`、`publish_base_tf:=false`、
  `cmd_vel_topic:=/cmd_vel`、`nav2_params_real.yaml`) で起動し、`sim:=true` でこれらがまとめて sim 用になります。
  個別に指定した引数は `sim` より優先されます。
* LiDAR の取付位置・傾き (前に 30 度 → `lidar_pitch_deg: 30`)、自車体の箱は
  `src/common/lidar_bridge/config/robot.yaml` の `lidar_bridge` の項、地面の高さと障害物の判定は同じファイルの
  `obstacle_detector` の項で設定します。障害物の高さ判定は、この取付姿勢で
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
  床の点から `lidar_pitch_deg` / `lidar_roll_deg` / `ground_z` を推定して今の値と並べて表示します。
  ロボットから 1 m 以内に障害物の点があれば (車体や取付台が写っている)、
  それを含む `self_box_min` / `self_box_max` の例も表示します。
* `cmd_vel_converter` は Twist を `tan(δ) = L·ω / v` で舵角に、`sin(δ) = K·sin(φ − φ0)` でサーボ角に、
  `throttle = deadband + v / gain` でスロットルに変換します。値は `calibration_file`
  (calibrate_drive の `summary.yaml`) か `src/common/robot_operation/config/operation.yaml` から読みます。
* キャリブレーション後に遅い指令でモーターが止まってしまう (トルク不足) ときは、`operation.yaml` の
  `min_forward_throttle` / `min_backward_throttle` に「止まった状態から確実に動き出すスロットル」を設定します。
  動く間はこの値を下回りません (キャリブレーションの結果では上書きされません。0 で無効)。
* `/cmd_vel` が 0.5 秒途絶えるとスロットル 0 を送ります。
* LiDAR (L1 は `/dev/ttyUSB0`) と Arduino の USB シリアルが同じ名前にならないよう、
  `port:=/dev/serial/by-id/...` で指定するのが確実です。

### 決めた秒数だけ前進して点群を記録する (`drive_forward`)

上の 1, 2, 4 を起動した状態 (Nav2 は不要) で実行します。`/cmd_vel` に一定速度を送り、
`--bag` を付けると走り出す前から止まった後までの生データと Point-LIO の出力を記録します。

```bash
ros2 run robot_operation drive_forward --speed 0.3 --duration 5 --bag ~/bags/forward1
```

* Docker のコンテナで動かすときは、記録先をコンテナを終了しても消えない、マウントしている場所 (`/opt/common_ws/src` など) にします。
* Ctrl-C で中止すると速度 0 を送ってから終了します。
* Point-LIO が作った地図は、real.launch.py を Ctrl-C で終了したときに
  `src/common/point_lio_ros2/PCD/scans.pcd` に保存されます。

## 実機の走行キャリブレーション (`robot_operation calibrate_drive`)

実機を自動で走らせて、次の2つの対応を Point-LIO のオドメトリから計測します。

* スロットル (operator.ino の -100..100) と速度 [m/s]
* サーボ角 [deg] と前輪の舵角 [deg]

```bash
# ターミナル1, 2: LiDAR ドライバと Point-LIO (車体は静止させておく)
ros2 launch unitree_lidar_ros2 launch.py
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false

# ターミナル3: 計測 (半径 3 m ほどの空きが必要)
ros2 run robot_operation calibrate_drive --wheelbase <ホイールベース[m]> --lidar-x <後輪軸からLiDARまでの前方向距離[m]>
```

* 手順は2段階です。まずステアリングを中央にして各スロットルで直進し、同じだけ後退して戻ります。次に各サーボ角で、一定のスロットルのまま1周（または時間切れ）するまで旋回します。
* LiDAR は前に 30 度傾いていても、X 軸が前を向いていれば問題ありません。機体 X 軸を水平面に投影した向きをヨー角として使います。
* 舵角は自転車モデル `tan(δ) = L / R` で求めます。旋回半径 R は、LiDAR の位置から後輪軸中心の値に換算します (`--lidar-x`)。
* 結果は `~/drive_calib/<日時>/` に保存されます。生データ `raw.csv`, `segments.csv`、区間ごとの結果 `throttle_result.csv`, `steering_result.csv`、フィット結果 `summary.yaml` の5つです。
* `--analyze <ディレクトリ>` を付けると走行せず、記録済みのデータを別の `--wheelbase` / `--lidar-x` で解析し直せます。
* Ctrl-C、またはオドメトリが 0.5 秒途絶えた場合は、スロットル 0・サーボ中央を送って止めます。ただし Arduino 側にはタイムアウトがないので、USB が抜けたときに備えてすぐ止められる状態で使ってください。
* 主なオプション: `--throttles 15,20,30`, `--servos 40,55,70,85,100,115,130`, `--steer-throttle 25`, `--max-distance 3.0`, `--pause` (区間ごとに Enter 待ち)。一覧は `--help` で確認できます。

## 一括起動と目的地の指定 (`robot_bringup` / `robot_goal`)

LiDAR ドライバ、lidar_bridge + Point-LIO、Nav2、Arduino 変換、2D 地図 (`robot_map`)、
スマホ用の Web サーバー (`robot_goal web`) を 1 つのコマンドで起動します。

```bash
ros2 launch robot_bringup robot.launch.py                       # 機種は $ROS2_TARGET (無ければ l2)
ros2 launch robot_bringup robot.launch.py lidar:=l1 lidar_port:=/dev/serial/by-id/... \
    arduino_port:=/dev/serial/by-id/... calibration_file:=$HOME/drive_calib/<日時>/summary.yaml
ros2 launch robot_bringup robot.launch.py dry_run:=true         # Arduino に送らずログだけ
ros2 launch robot_bringup robot.launch.py nav:=false operation:=false   # 地図作り・確認だけ
```

* `driver` / `point_lio` / `nav` / `operation` / `map` / `web` / `rviz` を `false` にすると
  その部分を起動しません (`rviz` は既定で `false`。PC 側で開く)。
* Nav2 は Point-LIO の TF が出始めるのを待つため `nav_delay` 秒 (既定 5) 後に起動します。

目的地は別のターミナルから送ります (座標は `map`、Point-LIO の起動位置が原点)。

```bash
ros2 run robot_goal goal 3.0 1.5        # (3.0, 1.5) へ。向きは今のまま
ros2 run robot_goal goal 3.0 1.5 90     # 到着時の向き [deg]
ros2 run robot_goal goal --rel 2.0 0    # 今の位置・向きから見て前に 2 m
ros2 run robot_goal goal                # 対話モード (h でヘルプ, c で取り消し, p で現在地)
```

到着・失敗まで残り距離を表示し、Ctrl-C でゴールを取り消して終了します。

### Arduino を挿したら自動で起動する (`install-autostart.sh`)

Arduino (`/dev/ttyACM0`) が現れたら上の `robot.launch.py` を起動し、抜いたら止めます。
udev が systemd のサービス `robot-bringup.service` を起動する仕組みです。

```bash
./scripts/install-autostart.sh l2          # sudo は付けない (ロボットを動かすユーザーで実行). L1 なら l1
./scripts/install-autostart.sh uninstall   # 元に戻す
```

* 設定は `/etc/default/robot-bringup` に書きます (ワークスペースの場所、`ROS_DOMAIN_ID`、
  launch 引数 `ROBOT_LAUNCH_ARGS="calibration_file:=... web_token:=..."` など)。
  変更したら `sudo systemctl restart robot-bringup` するか、Arduino を挿し直します。
* ログは `journalctl -u robot-bringup -f`、状態は `systemctl status robot-bringup` で見ます。
* systemd から起動したプロセスは `~/.bashrc` を読みません。そのため
  `scripts/autostart/robot-bringup.sh` が ROS とワークスペースの `setup.bash` を読み込んでから起動します。
* 止めるときは `ros2 launch` にだけ SIGINT を送り、各ノードは launch が順に止めます (Ctrl-C と同じ)。
* 起動に失敗したら 10 秒後にやり直します (LiDAR やネットワークの準備がまだのとき)。

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
* モードを止めるたびに、Point-LIO が保存した地図 (`PCD/scans.pcd`) を `~/robot_maps/<日時>_<モード>.pcd` に移します
  (次のモードで上書きされないように)。
* 自動起動 (`scripts/start_ros.sh`、`scripts/autostart/robot-bringup.sh`) はこの `launcher` を起動します。
  モードを順に止める時間が要るので、サービスの `TimeoutStopSec` は 60 秒程度にしてください。

```bash
ros2 run robot_launcher launcher                                   # 手で起動する場合
ros2 run robot_launcher launcher --ros-args -p token:=abcd -p launch_args:="lidar_port:=/dev/serial/by-id/..."
```

## スマホから地図をタップして目的地を送る (`robot_map` / `robot_goal web`)

走りながら 2D 地図を作り、同じ Wi-Fi につないだスマホのブラウザでその地図をタップして目的地を送ります。
2D 地図を作るのは `robot_map` だけで、Web 画面、RViz (`Map2D` 表示)、必要なら Nav2 がその `/map` を共有します。

![LiDAR から地図・Web・走行までのデータの流れ](docs/images/data_flow.svg)

線の色はデータの種類 (橙: 3D 点群、青: 2D 地図・経路、灰: IMU・オドメトリ・指令、緑: HTTP)、
破線は任意またはファイル経由です。トピック名は `real.launch.py` で起動したときの実際の名前です。

### 起動

`robot.launch.py` で一緒に起動します (既定で `map:=true web:=true`)。起動ログに
`スマホで開く: http://192.168.x.x:8080` と出るので、そのアドレスをスマホで開きます。

```bash
ros2 launch robot_bringup robot.launch.py                        # 地図作り + Web
ros2 launch robot_bringup robot.launch.py web_token:=abcd        # http://<IP>:8080/?key=abcd の人だけ操作できる
ros2 launch robot_bringup robot.launch.py map_pcd:=$HOME/maps/scans.pcd   # 保存済みの地図から始める
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
| 追従 / 全体 | ロボットを中心に表示 / 観測済みの範囲を全部表示 |

上の帯に、地図・現在地の受信状況、Nav2 の接続、走行状態と残り距離が出ます。

Nav2 の costmap を見るときは、表示する地図を差し替えた 2 つ目の Web サーバーを起動します
(`http://<IP>:8090`)。黒が障害物、赤がロボットの中心が入るとぶつかる範囲、
オレンジが障害物から離れたい範囲 (濃いほど近い) です。

```bash
ros2 run robot_goal web --ros-args -r __node:=costmap_web -p map_topic:=/global_costmap/costmap -p port:=8090
```

### 地図の作り方 (`robot_map`)

Point-LIO の `/cloud_registered` (map 座標の点群、毎スキャン) を base_link 座標に直して高さで分け、
5 cm 四方のセルごとに数えます。

* 床から ±`floor_tolerance` (5 cm) の点 → 床、`obstacle_min_height`〜`obstacle_max_height` の点 → 障害物。
  `ground_z` と障害物の高さは `robot.yaml` (`obstacle_detector` の項) と同じ値を使います。
* 障害物の点が 3 回以上で、かつ床の点の半分以上ならそのセルは占有、床だけなら空き、どちらも無ければ未知です。
  人が横切った跡は、その後に床がたくさん見えれば空きに戻ります。
* ロボットから `max_range` (15 m) より遠い点は使いません。
* `/map` は変化があったときに 1 秒ごとに配信します (transient_local なので、後から起動したノードも受け取れます)。
* `pcd_file` を指定すると、保存済みの地図 (Point-LIO の `scans.pcd` など) を最初に読み込みます。
  床の高さは点の高さの分布から自動で推定します (`pcd_ground_z` で指定も可)。
* Nav2 は今のところ `/map` を使いません (global costmap は rolling window のまま)。


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

パラメータは `src/common/point_lio_ros2/config/*.yaml` にあります。

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
2. `config/unilidar_l1.yaml` の調整 (変更前の控えが `unilidar_l1.yaml.bak`)
3. `mapping_unilidar_l1.launch.py` / `mapping_unilidar_l2.launch.py` に `use_sim_time` 引数 (Gazebo / bag 再生用)
4. RViz の表示設定

上流の `image/` (README 用のデモ GIF、約 246 MB) と、実体のないサブモジュール定義の `.gitmodules` は取りません
(`include/IKFoM` と `include/ikd-Tree` は上流でも実ファイルとしてコミット済み)。

Livox AVIA の `CustomMsg` 入力経路は上流でもコメントアウトされており
(`livox_ros_driver2` を含めていないため)、標準の `sensor_msgs/PointCloud2` 経路のみが有効です。

## Docker で動かす (任意)

Docker イメージは L1 / L2 / sim (Gazebo) の 3 種類です。ビルドの前に外部リポジトリを取得しておきます
(イメージには `src/` の中身がそのまま入るため。GitHub Actions のビルドでは自動で実行します)。

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

* **Gazebo Classic / gazebo_ros_pkgs**
  * **License:** [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)
  * **Copyright:** Copyright Open Source Robotics Foundation, Inc.
  * **Source:** [ros-simulation/gazebo_ros_pkgs (GitHub)](https://github.com/ros-simulation/gazebo_ros_pkgs)
