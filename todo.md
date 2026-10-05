## 1. パラメータのドキュメント
|内容|リンク|
|---|---|
|設定ガイドの目次|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/|
|costmap 共通（resolution, rolling_window, footprint など）	|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/|
|ObstacleLayer|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/|
|VoxelLayer|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/|
|InflationLayer|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/|
|Smac Hybrid|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/|
|planner_server|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/|
|Regulated Pure Pursuit|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/|
|controller_server|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/|
|SimpleGoalChecker|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/controller_server_plugins/simple_goal_checker/|
|behavior_server|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/|
|bt_navigator|https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/|
|調整のコツ（チューニングガイド）|https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/|
  
## 2. その他
|内容|リンク|
|---|---|
|Ackermann コントローラ（Humble 版）|https://control.ros.org/humble/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html|
|ステアリング系コントローラ共通のパラメータ（Humble 版）|https://control.ros.org/humble/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html|
|SDF の <sensor>（LiDAR / IMU の設定）|http://sdformat.org/spec?ver=1.8&elem=sensor|
|pcl_ros|https://docs.ros.org/en/humble/p/pcl_ros/|

---

## BT の基本
BT は、木の根から一定の周期（bt_navigator の既定は 100 Hz）で tick（実行の合図） を子ノードへ順に送って動きます。
各ノードは tick されるたびに、次の3つのどれかを返します。
SUCCESS：成功
FAILURE：失敗
RUNNING：実行中（まだ終わっていない）
ノードは役割で4種類に分かれます。
制御ノード：子をどの順番でどう実行するかを決める
デコレータ：子を1つだけ持ち、その結果を加工する
条件ノード：判定だけを行う
アクションノード：実際に何かをする
{goal} や {path} は ブラックボード（ノード間で共有する変数置き場）の変数です。
name="..." はログや可視化ツール（Groot）で表示するためのラベルで、動作には影響しません。
木の全体像
RecoveryNode (6回まで)                       ← 失敗したらリカバリして再挑戦
├─ PipelineSequence                          ← 本体：計画と追従を並行して進める
│   ├─ RateController (1 Hz)                 ← 1秒に1回だけ判定
│   │   └─ Fallback                          ← 経路を維持するか、計画し直すか
│   │       ├─ ReactiveSequence              ← 「維持してよいか」の判定
│   │       │   ├─ Inverter
│   │       │   │   └─ GlobalUpdatedGoal     ← ゴールが変わったか
│   │       │   └─ IsPathValid               ← 今の経路は通れるか
│   │       └─ RecoveryNode (1回)
│   │           ├─ ComputePathToPose         ← 経路を計画する
│   │           └─ ClearEntireCostmap (global)
│   └─ RecoveryNode (1回)
│       ├─ FollowPath                        ← 経路に沿って走る
│       └─ ClearEntireCostmap (local)
└─ ReactiveFallback                          ← リカバリ側
    ├─ GoalUpdated                           ← 新しいゴールが来たら、リカバリを中断
    └─ RoundRobin                            ← リカバリ動作を順番に1つずつ
        ├─ Sequence (costmap を両方クリア)
        ├─ BackUp
        └─ Wait
各タグの解説
<root main_tree_to_execute="MainTree"> と <BehaviorTree ID="MainTree">

XML の最上位です。BehaviorTree に ID を付けて木を定義し、root で「どの ID の木を実行するか」を指定します。1つのファイルに複数の木を書けるので、こういう仕組みになっています。

<RecoveryNode number_of_retries="6">（一番外側、制御ノード）

子を2つ持ちます。

1つ目（本体）が SUCCESS → そのまま SUCCESS（ナビゲーション完了）。
1つ目が FAILURE → 2つ目（リカバリ）を実行します。リカバリが SUCCESS なら、1つ目を最初からやり直します。
やり直しは number_of_retries 回まで。それを超えるか、リカバリ自体が FAILURE ならナビゲーション失敗です。
RUNNING はそのまま上に返します。
<PipelineSequence>（制御ノード）

子を順番に実行しますが、普通の Sequence とは違って、一度進んだ後も、前の子を毎回 tick し続けます。

1つ目（計画側）が一度 SUCCESS を返して経路ができたら、2つ目（FollowPath）が走り始めます。
その後も、毎 tick で1つ目を実行してから2つ目を実行します。これで「走りながら、裏で経路をチェックする」動きになります。
どれかの子が FAILURE を返したら、全体が FAILURE になります。
最後の子（FollowPath）が SUCCESS になったら、全体が SUCCESS になります。
<RateController hz="1.0">（デコレータ）

子を tick する頻度を制限します。

前回から 1 秒経っていなければ子を実行せず、RUNNING を返します。PipelineSequence は、すでに先の子に進んでいるときは前の子の RUNNING を無視して次に進むので、走行は止まりません。
子が RUNNING の間（計画中）は、計画が終わるまで毎回 tick します。
<Fallback>（制御ノード）

子を順に試し、最初に SUCCESS を返した子で止まります。いわば「A がダメなら B」です。

1つ目（「今の経路を維持してよいか」の判定）が SUCCESS なら、計画はしません。ここで経路が維持されます。
1つ目が FAILURE のときだけ、2つ目（計画）を実行します。
<ReactiveSequence>（制御ノード）

子が全部 SUCCESS なら SUCCESS、1つでも FAILURE なら FAILURE です（AND 条件）。tick のたびに先頭の子から評価し直します。ここでは「ゴールが変わっていない」かつ「経路が通れる」を判定しています。

<Inverter>（デコレータ）

子の SUCCESS と FAILURE を入れ替えます。

<GlobalUpdatedGoal/>（条件ノード）

ブラックボードの goal（または goals）が、前回見たときから変わっていれば SUCCESS です。

一番最初の tick では必ず SUCCESS を返します。 そのため、Inverter を通すと FAILURE になり、初回は必ず計画が走ります。
新しいゴールが来たら SUCCESS → Inverter で FAILURE → 計画し直します。
<IsPathValid path="{path}"/>（条件ノード）

planner_server の is_path_valid サービスを呼びます。

経路が空のとき → FAILURE。
車両に最も近い点から先の各点に footprint を置いて、どれか1つでも障害物（LETHAL）に重なったら → FAILURE。
インフレーション（灰色の領域）にかかるだけでは無効になりません。本当にぶつかるときだけ計画し直すので、左右の迷いが起きにくくなります。
<RecoveryNode number_of_retries="1" name="ComputePathToPose">（計画側）

計画に失敗したら、global costmap をクリアして1回だけやり直します。

<ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>（アクションノード）

planner_server に「{goal} までの経路を計画して」と依頼し、結果を {path} に書き込みます。planner_id は nav2_params.yaml の planner_plugins に書いた名前（GridBased = Smac Hybrid）です。計画中は RUNNING、成功で SUCCESS、失敗で FAILURE を返します。

<ClearEntireCostmap service_name="..."/>（アクションノード）

指定した costmap を全部消去するサービスを呼びます。古い障害物の情報が残っていて計画や追従ができないときの、最も軽いリカバリです。

<RecoveryNode number_of_retries="1" name="FollowPath">（追従側）

追従に失敗したら、local costmap をクリアして1回だけやり直します。

<FollowPath path="{path}" controller_id="FollowPath"/>（アクションノード）

controller_server（RPP）に「{path} に沿って走って」と依頼します。

走っている間は RUNNING、ゴールに着いたら SUCCESS、止まって進めなくなったら FAILURE です。
走行中に {path} が書き換わると、新しい経路をコントローラに送り直します（follow_path_action.cpp の on_wait_for_result）。計画し直した経路は、止まらずにそのまま反映されます。
<ReactiveFallback>（リカバリ側、制御ノード）

Fallback と同じく「最初に SUCCESS を返した子で止まる」ノードですが、tick のたびに先頭の子から評価し直します。そのため、リカバリ動作の最中でも、毎回まず GoalUpdated を確認できます。

<GoalUpdated/>（条件ノード）

このノードが前回 tick されたときからゴールが変わっていれば SUCCESS です。リカバリ中（後退中や待機中）に RViz で新しいゴールを置くと、ここが SUCCESS になります。リカバリを中断して、すぐに本体の再挑戦に移ります。

<RoundRobin>（制御ノード）

呼ばれるたびに、前回の次の子を1つだけ実行します。

1回目のリカバリ → costmap のクリア
2回目 → 後退
3回目 → 待機
4回目 → またクリア……と巡回します。

子が FAILURE なら、同じ回の中で次の子を試します。全部 FAILURE なら FAILURE を返します。

<Sequence name="ClearingActions">（制御ノード）

子を順に実行し、全部 SUCCESS なら SUCCESS です。local と global の costmap を続けてクリアします。

<BackUp backup_dist="0.5" backup_speed="0.1"/>（アクションノード）

behavior_server に「0.1 m/s で 0.5 m 後退して」と依頼します。ステアはまっすぐなので、アッカーマン車でも実行できます。後ろに障害物があると途中で止まり、FAILURE になります。

<Wait wait_duration="3"/>（アクションノード）

3 秒待ちます。人や動く障害物が通り過ぎるのを待ち、その間に costmap が更新されることを期待する動作です。

既定の BT から外したもの：<Spin spin_dist="1.57"/>（その場で 90° 旋回）。アッカーマン車はその場で回れず、毎回時間切れで失敗していたため外しました。

実際の流れ
最初の tick
GlobalUpdatedGoal が初回なので SUCCESS → Inverter で FAILURE になり、経路を維持する条件が成り立ちません。
→ ComputePathToPose で計画し、{path} ができます。
→ FollowPath が走り出します。
走行中（1 秒ごと）
ゴールが変わっておらず、経路上に障害物がなければ、経路を維持します（計画しない）。
経路上に障害物が入ったら計画し直し、FollowPath に新しい経路が届きます。
ゴールに到着 → FollowPath が SUCCESS → 全体が SUCCESS。
途中で詰まった場合
FollowPath が FAILURE → local costmap をクリアして1回やり直し → それでもダメなら本体が FAILURE。
→ リカバリを1つ実行（クリア → 後退 → 待機の順に巡回） → 本体を最初からやり直し。
これを最大 6 回繰り返します。

各ノードの公式の説明は、以下にあります（ノード名のページへのリンク付き）。
https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/

ただし、Rolling（開発版）のドキュメントなので、Humble にはないノードや属性も載っています。