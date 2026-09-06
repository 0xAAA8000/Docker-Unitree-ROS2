# ROS 2 コマンド チートシート (Humble)

このリポジトリの Docker イメージ (`osrf/ros:humble-desktop-full` ベース) で使うことを
前提にしたコマンド集です。例はすべて、このワークスペースに入っている
`unitree_lidar_ros2` と `point_lio` の実際のノード名・トピック名を使っています。

- [0. 基本の型](#0-基本の型)
- [1. まず覚える 8 個](#1-まず覚える-8-個)
- [2. ノードを起動する — `run` / `launch`](#2-ノードを起動する--run--launch)
- [3. トピック — `topic`](#3-トピック--topic)
- [4. ノード — `node`](#4-ノード--node)
- [5. パラメータ — `param`](#5-パラメータ--param)
- [6. サービス — `service`](#6-サービス--service)
- [7. 記録と再生 — `bag`](#7-記録と再生--bag)
- [8. メッセージ型 — `interface`](#8-メッセージ型--interface)
- [9. パッケージ — `pkg`](#9-パッケージ--pkg)
- [10. 座標変換 — TF](#10-座標変換--tf)
- [11. GUI ツール](#11-gui-ツール)
- [12. ビルド — `colcon`](#12-ビルド--colcon)
- [13. 環境変数とデーモン](#13-環境変数とデーモン)
- [14. トラブルシュート早見表](#14-トラブルシュート早見表)
- [15. このリポジトリでの定番コマンド](#15-このリポジトリでの定番コマンド)

---

## 0. 基本の型

ROS 2 の CLI はすべてこの形です。

```
ros2 <コマンド> <サブコマンド> [引数]
```

迷ったらこの 3 つ。

```bash
ros2 --help                  # コマンド一覧
ros2 topic --help            # サブコマンド一覧
ros2 topic echo --help       # オプション一覧
```

**Tab 補完が効きます。** `ros2 topic <Tab><Tab>` でサブコマンドが、
`ros2 topic echo <Tab>` で実際に存在するトピック名が補完されます。
覚えるより補完に頼るほうが速いです。

---

## 1. まず覚える 8 個

実務ではこれでほぼ足ります。

| コマンド | 何をするもの | 代表例 |
| --- | --- | --- |
| `ros2 run` | ノードを 1 個起動 | `ros2 run rviz2 rviz2` |
| `ros2 launch` | 複数ノードをまとめて起動 | `ros2 launch point_lio mapping_unilidar_l2.launch.py` |
| `ros2 topic` | トピックを見る / 流す | `ros2 topic hz /unilidar/cloud` |
| `ros2 node` | ノードの状態を見る | `ros2 node info /laserMapping` |
| `ros2 param` | パラメータを読み書き | `ros2 param get /laserMapping mapping.det_range` |
| `ros2 bag` | データを録る / 再生する | `ros2 bag record -a` |
| `ros2 interface` | メッセージの型定義を見る | `ros2 interface show sensor_msgs/msg/Imu` |
| `ros2 pkg` | パッケージを探す | `ros2 pkg list \| grep point_lio` |

---

## 2. ノードを起動する — `run` / `launch`

### `ros2 run` — 1 個だけ起動

```bash
ros2 run <パッケージ名> <実行ファイル名>
```

```bash
ros2 run point_lio pointlio_mapping
ros2 run rviz2 rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

実行ファイル名が分からないとき:

```bash
ros2 pkg executables point_lio
```

**引数は `--ros-args` の後ろに書きます。** ROS 1 と最も違う点です。

```bash
# パラメータを個別指定
ros2 run point_lio pointlio_mapping --ros-args -p use_sim_time:=true

# 設定ファイルを読ませる
ros2 run point_lio pointlio_mapping --ros-args \
  --params-file /ros2_ws/src/point_lio_ros2/config/unilidar_l2.yaml

# トピック名を差し替える (リマップ)
ros2 run point_lio pointlio_mapping --ros-args -r /pointlio/odom:=/my_odom

# ノード名を変える
ros2 run point_lio pointlio_mapping --ros-args -r __node:=my_lio

# ログレベルを変える
ros2 run point_lio pointlio_mapping --ros-args --log-level debug
```

複数まとめて書けます。

```bash
ros2 run point_lio pointlio_mapping --ros-args \
  --params-file cfg.yaml \
  -p use_sim_time:=true \
  -r /velodyne_points:=/unilidar/cloud
```

### `ros2 launch` — まとめて起動

```bash
ros2 launch <パッケージ名> <launchファイル> 引数:=値
```

```bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false use_sim_time:=true
```

launch ファイルを直接パス指定することもできます。

```bash
ros2 launch /ros2_ws/src/point_lio_ros2/launch/mapping_unilidar_l2.launch.py
```

| オプション | 意味 |
| --- | --- |
| `-s` / `--show-args` | **受け付ける引数の一覧を表示** (最初にこれを打つと事故が減る) |
| `-p` / `--print` | 実行せず、何が起動されるかを表示 |
| `-d` / `--debug` | 詳細ログ |
| `-n` / `--noninteractive` | 対話なし (スクリプト用) |
| `-a` / `--show-all-subprocesses-output` | 全子プロセスの出力を表示 |

```bash
ros2 launch point_lio mapping_unilidar_l2.launch.py -s
```

---

## 3. トピック — `topic`

一番よく使うコマンド群です。

```bash
ros2 topic list                       # 一覧
ros2 topic list -t                    # 型付きで一覧
ros2 topic type /unilidar/cloud       # 型だけ知りたいとき
ros2 topic find sensor_msgs/msg/Imu   # 型からトピックを探す
ros2 topic info /unilidar/cloud -v    # publisher/subscriber と QoS (-v が重要)
ros2 topic hz /unilidar/cloud         # 周波数
ros2 topic bw /unilidar/cloud         # 帯域
ros2 topic delay /unilidar/cloud      # ヘッダのタイムスタンプと現在時刻の差
```

**`ros2 topic hz` は「データが流れているか」を確認する最短の方法です。**
何かおかしいときは、まずこれを打ちます。

### `echo` — 中身を見る

点群のような巨大メッセージをそのまま `echo` すると端末が埋まります。
次のオプションを使ってください。

```bash
ros2 topic echo /pointlio/odom                       # ふつうに表示
ros2 topic echo /unilidar/cloud --no-arr             # 配列を省略 (点群はこれ)
ros2 topic echo /unilidar/cloud --field fields       # 特定フィールドだけ
ros2 topic echo /pointlio/odom --once                # 1 回だけ表示して終了
ros2 topic echo /pointlio/odom --csv                 # CSV 形式
ros2 topic echo /pointlio/odom --truncate-length 20  # 長い値を切り詰める
ros2 topic echo /diagnostics --filter "m.level > 0"  # 条件で絞る
```

`--field fields` は、点群に `ring` や `time` フィールドがあるかを調べるのに便利です
(Point-LIO を Gazebo で回すときの確認に使います)。

### QoS が合わないとき

センサ系トピックは `Best Effort` で配信されることが多く、
既定の `Reliable` で購読すると **何も受信できません**。

```bash
ros2 topic echo /unilidar/cloud --qos-reliability best_effort --no-arr
ros2 topic echo /map --qos-durability transient_local
```

### `pub` — 手で値を流す

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"   # 10Hz
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"      # 停止
```

YAML の書き方が分からないときは、ひな形を生成できます。

```bash
ros2 interface proto geometry_msgs/msg/Twist
```

---

## 4. ノード — `node`

```bash
ros2 node list
ros2 node list -a               # 隠しノードも含める
ros2 node info /laserMapping    # 購読/配信トピック、サービス、パラメータを一覧
```

`ros2 node info` は「このノードは何を待っていて、何を出すのか」が
一度に分かるので、繋ぎ込みで迷ったときに効きます。

---

## 5. パラメータ — `param`

```bash
ros2 param list                                        # 全ノード
ros2 param list /laserMapping                          # 特定ノード
ros2 param get /laserMapping mapping.det_range
ros2 param set /laserMapping mapping.det_range 50.0    # 実行中に変更
ros2 param describe /laserMapping mapping.det_range    # 型や説明
ros2 param dump /laserMapping                          # 現在値を YAML で出力
ros2 param load /laserMapping params.yaml              # YAML から流し込む
```

### ROS 2 のパラメータで引っかかりやすい点

1. **区切りは `/` ではなく `.`**
   ROS 1 の `mapping/det_range` は ROS 2 では `mapping.det_range` です。

2. **型が厳密**
   `double` として宣言されたパラメータに YAML で `35` (整数) を書くと
   型不一致でエラーになります。`35.0` と書く必要があります。

3. **事前宣言が必須**
   ノードが `declare_parameter` していないパラメータは `-p` で渡しても
   受け付けられません (`ros2 param list` に出ないものは設定できません)。

4. **YAML の形式**

```yaml
/**:                        # ← ワイルドカード。どのノード名でも適用される
    ros__parameters:
        mapping:
            det_range: 100.0
```

`/**` の代わりにノード名 (`laserMapping:`) を書くと、そのノード限定になります。

---

## 6. サービス — `service`

```bash
ros2 service list
ros2 service list -t                                   # 型付き
ros2 service type /spawn_entity
ros2 service find std_srvs/srv/Empty
ros2 service call /reset_simulation std_srvs/srv/Empty  # 呼び出す
```

Gazebo の制御 (一時停止、リセット、モデル配置) はサービス経由です。

```bash
ros2 service call /pause_physics std_srvs/srv/Empty
ros2 service call /unpause_physics std_srvs/srv/Empty
```

---

## 7. 記録と再生 — `bag`

**実機がなくてもアルゴリズムを何度でも試せるので、一番実用的な機能です。**

### 録る

```bash
ros2 bag record -a                                       # 全トピック
ros2 bag record /unilidar/cloud /unilidar/imu            # 指定トピックのみ (推奨)
ros2 bag record -o my_run /unilidar/cloud /unilidar/imu  # 出力名を指定
ros2 bag record -e "/unilidar/.*"                        # 正規表現で選ぶ
ros2 bag record -a -x "/camera/.*"                       # 除外
ros2 bag record -a -b 2000000000                         # 2GB ごとに分割
ros2 bag record -a -s mcap                               # mcap 形式で保存
```

点群は非常に重いので、`-a` ではなく必要なトピックだけ指定するのが基本です。

### 見る / 再生する

```bash
ros2 bag info my_run

ros2 bag play my_run
ros2 bag play my_run --clock              # /clock を配信 (use_sim_time とセット)
ros2 bag play my_run -r 0.5               # 半速
ros2 bag play my_run -l                   # ループ
ros2 bag play my_run -p                   # 一時停止状態で開始 (Space で再生)
ros2 bag play my_run --topics /unilidar/cloud /unilidar/imu
ros2 bag play my_run --start-offset 30    # 30 秒目から
ros2 bag play my_run --remap /old:=/new
```

**`--clock` と `use_sim_time:=true` は必ずセットで使ってください。**
片方だけだとノードが時刻を待って固まります。

```bash
# ターミナル1
ros2 bag play my_run --clock
# ターミナル2
ros2 launch point_lio mapping_unilidar_l2.launch.py use_sim_time:=true
```

---

## 8. メッセージ型 — `interface`

```bash
ros2 interface show sensor_msgs/msg/Imu           # 型定義を表示
ros2 interface show sensor_msgs/msg/PointCloud2
ros2 interface list | grep -i pointcloud          # 型を探す
ros2 interface package sensor_msgs                # パッケージ内の型一覧
ros2 interface proto geometry_msgs/msg/Twist      # topic pub 用のひな形
```

---

## 9. パッケージ — `pkg`

```bash
ros2 pkg list
ros2 pkg list | grep point_lio
ros2 pkg prefix point_lio            # インストール先パス
ros2 pkg executables point_lio       # 実行ファイル一覧
ros2 pkg xml point_lio               # package.xml を表示
```

設定ファイルや launch ファイルの実体を探すとき:

```bash
ls $(ros2 pkg prefix point_lio)/share/point_lio/config/
ls $(ros2 pkg prefix point_lio)/share/point_lio/launch/
```

---

## 10. 座標変換 — TF

`ros2 tf` というコマンドは **ありません**。`ros2 run` 経由で使います。

```bash
ros2 run tf2_tools view_frames                        # TF ツリーを PDF に出力
ros2 run tf2_ros tf2_echo camera_init aft_mapped      # 2 フレーム間の変換を表示
ros2 run tf2_ros tf2_monitor                          # 遅延などを監視
ros2 topic echo /tf_static --once                     # 静的 TF を確認
```

固定の座標変換を手で流す (Humble では名前付き引数):

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.4 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id velodyne
```

> **RViz で何も表示されないときの最頻原因は Fixed Frame の間違いです。**
> Point-LIO なら `camera_init` を指定してください。

---

## 11. GUI ツール

```bash
ros2 run rqt_graph rqt_graph        # ノードとトピックの接続図 (原因究明に最強)
ros2 run rqt_console rqt_console    # ログビューア
ros2 run rqt_reconfigure rqt_reconfigure   # パラメータを GUI で変更
rqt                                 # 全部入り
rviz2                               # 3D 可視化
```

`rqt_graph` は「繋がっているつもりで繋がっていない」を一目で見つけられます。

---

## 12. ビルド — `colcon`

`colcon` は `ros2` コマンドではなく独立したビルドツールです。
**必ずワークスペースのルート (`/ros2_ws`) で実行してください。**

```bash
cd /ros2_ws

colcon build                                    # 全部
colcon build --packages-select point_lio        # 1 つだけ (普段はこれ)
colcon build --packages-up-to point_lio         # 依存も含めて
colcon build --symlink-install                  # yaml/launch の編集がビルド無しで反映
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --event-handlers console_direct+   # コンパイルログを全部出す (エラー調査)
colcon build --continue-on-error                # 失敗しても他を続行

source install/setup.bash                       # ビルド後は必ずこれ
```

きれいにやり直したいとき:

```bash
rm -rf build install log && colcon build
```

> **「パッケージが見つからない」の大半は `source install/setup.bash` の忘れです。**
> このイメージでは `entrypoint.sh` が起動時に自動で source しますが、
> コンテナ内で再ビルドしたあとは手動で source し直す必要があります。

---

## 13. 環境変数とデーモン

```bash
echo $ROS_DISTRO             # humble
echo $ROS_DOMAIN_ID          # 既定は 0。値が違うと互いのノードが見えない
echo $RMW_IMPLEMENTATION     # DDS 実装。異なると通信できないことがある
printenv | grep ROS

ros2 doctor                  # 環境診断
ros2 doctor --report         # 詳細レポート
```

複数マシン / 複数コンテナでノードが見えないときは、まず `ROS_DOMAIN_ID` を揃えます。

```bash
export ROS_DOMAIN_ID=42      # 全ターミナルで同じ値にする
```

`ros2 node list` が実態と食い違うときは、CLI のデーモンをリセットします。

```bash
ros2 daemon status
ros2 daemon stop && ros2 daemon start
```

---

## 14. トラブルシュート早見表

**上から順に潰してください。**

| # | 確認 | コマンド | 出なければ疑うところ |
| --- | --- | --- | --- |
| 1 | ノードは起きているか | `ros2 node list` | 起動失敗。端末のエラーを読む |
| 2 | トピックはあるか | `ros2 topic list` | トピック名の綴り、名前空間、リマップ |
| 3 | データは流れているか | `ros2 topic hz /xxx` | 配信側の設定 (デバイス、Gazebo のプラグイン) |
| 4 | 繋がっているか | `ros2 topic info /xxx -v` | `Subscription count: 0` なら **QoS 不一致** |
| 5 | 全体像 | `ros2 run rqt_graph rqt_graph` | 接続の切れ目を目で探す |
| 6 | 座標系 | `ros2 run tf2_tools view_frames` | TF が繋がっていない / Fixed Frame 違い |
| 7 | 環境 | `ros2 doctor --report` | `ROS_DOMAIN_ID`、DDS |

### よくある症状と原因

| 症状 | 原因 |
| --- | --- |
| パッケージが見つからない | `source install/setup.bash` 忘れ |
| RViz が真っ黒 / 何も出ない | Fixed Frame が違う (Point-LIO なら `camera_init`) |
| トピックはあるのに `echo` で何も出ない | QoS 不一致。`--qos-reliability best_effort` を試す |
| ノードが時刻待ちで固まる | `use_sim_time:=true` なのに `/clock` が来ていない |
| 別マシンのノードが見えない | `ROS_DOMAIN_ID` 不一致 |
| パラメータが反映されない | 型不一致 (`35` と `35.0`)、または区切りが `/` のまま |
| GUI が出ない (Docker) | `DISPLAY` 未転送。ホストで `xhost +local:root` |

---

## 15. このリポジトリでの定番コマンド

### 起動確認

```bash
ros2 pkg list | grep -E "point_lio|unitree_lidar|gazebo_ros"
ros2 topic list
```

### 実機 (Unitree L2) で Point-LIO

```bash
# ターミナル1
ros2 launch unitree_lidar_ros2 launch.py
# ターミナル2
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

確認:

```bash
ros2 topic hz /unilidar/cloud /unilidar/imu
ros2 topic hz /pointlio/odom
ros2 run tf2_ros tf2_echo camera_init aft_mapped
```

### Gazebo の動作確認 (LiDAR 付きサンプルがすぐ動きます)

```bash
ros2 launch gazebo_ros gazebo.launch.py                 # Gazebo 単体
ros2 launch velodyne_description example.launch.py      # VLP-16 付きロボット + RViz
ros2 launch velodyne_description example.launch.py gpu:=True
```

```bash
ros2 topic hz /velodyne_points
ros2 topic echo /velodyne_points --field fields   # ring / intensity の確認
```

### データを録って後で試す

```bash
ros2 bag record -o run1 /unilidar/cloud /unilidar/imu
# 後日
ros2 bag play run1 --clock
ros2 launch point_lio mapping_unilidar_l2.launch.py use_sim_time:=true
```

### 地図を保存する

`config/*.yaml` の `pcd_save.pcd_save_en` が `true` なら、終了時に
`src/point_lio_ros2/PCD/scans.pcd` へ保存されます。
**コンテナを消すと失われる**ので、残したい場合はホストのディレクトリを
マウントしてください。

```bash
docker run -it --rm \
  -v $(pwd)/output:/ros2_ws/src/point_lio_ros2/PCD \
  ... <イメージ>
```

---

## 参考

- [ROS 2 Humble 公式チュートリアル](https://docs.ros.org/en/humble/Tutorials.html)
- [CLI ツール入門 (公式チュートリアル)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [QoS の設定](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

このチートシートに載っているオプションは、Humble の `ros2cli` / `rosbag2` /
`launch_ros` / `geometry2` のソースで実在を確認したものです。
バージョンによって差があるため、迷ったら `--help` を優先してください。
