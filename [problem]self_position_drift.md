# 自己位置が予測しない方向にずれていく問題の解決



## 試験
### 試験内容
1. 水平面に接地させる。地面はゴム
2. 手で持ち、動かさない。1に対し、細かな揺れの影響を調べる。
3. 手で持ち、xyz方向にゆっくり動かす。
4. 手で持ち、xyz軸の回転をする。
5. 手で持ち、xyzrpyを混ぜて動かす。

```bash
$ ros2 run lidar_bridge check_inputs --ros-args -p cloud:=/unilidar/cloud -p imu:=/unilidar/imu
[INFO] [1791604322.574903089] [check_lio_inputs]: cloud=/unilidar/cloud imu=/unilidar/imu  車両を静止させたまま待つこと
[INFO] [1791604327.555532633] [check_lio_inputs]: ---- IMU ----
[INFO] [1791604327.556178021] [check_lio_inputs]:   rate=216.6 Hz dt[mean=+0.0046 min=+0.0001 max=+0.0200] stamp逆行=0
[INFO] [1791604327.556789429] [check_lio_inputs]:   acc mean=[+0.002 +0.129 +9.915] |mean|=9.916 std=[0.117 0.115 0.196]
[INFO] [1791604327.557338028] [check_lio_inputs]:   gyr mean=[+0.0041 +0.0003 +0.0040] rad/s std=[0.0312 0.0175 0.0034]
[INFO] [1791604327.557858336] [check_lio_inputs]:   重力方向と IMU z 軸の角度=0.7 deg (取付 pitch と合うか)
[INFO] [1791604327.558318495] [check_lio_inputs]:   受信遅れ[mean=+0.0004 min=+0.0002 max=+0.0176] s
[INFO] [1791604327.558802393] [check_lio_inputs]: ---- Cloud ----
[INFO] [1791604327.559441021] [check_lio_inputs]:   rate=8.3 Hz points mean=2088
[INFO] [1791604327.559924600] [check_lio_inputs]:   time field min[mean=+0.0000 min=+0.0000 max=+0.0000] max[mean=+0.1170 min=+0.1024 max=+0.1358] s
[INFO] [1791604327.560350059] [check_lio_inputs]:   スキャン終了時刻 - 最新IMU stamp [mean=-0.0007 min=-0.0033 max=+0.0033] s  (0 付近のはず)
[INFO] [1791604327.560864437] [check_lio_inputs]:   受信遅れ[mean=+0.1208 min=+0.1053 max=+0.1393] s
```


### デフォルト
#### 試験1
| pram | value |
| --- | --- |
| imu_meas_acc_cov | 0.1 |
| acc_cov_output | 1000.0 |
| imu_meas_omg_cov | 0.1 |
| gyr_cov_output | 1000.0 |
| init_map_size | 1000 |

1. 水平面に接地、地面は振動しにくいゴム。
    * 5分間おいてみたが、ドリフトは発生しなかった。
2. 手で持ち、動かさない。
    * 自己位置はおおむね正しく推測されていた。
3. 手で持ち、xyz方向にゆっくり動かす。
    * ある程度動かしても問題なく自己位置の推定ができていた。
    * 元の場所に戻ってきた際も、画面上も元の場所に戻れていた。
4. 手で持ち、xyz軸の回転を行う。
    * x軸方向の回転をすると、自己位置が大きくずれ始める挙動が見れた。
    * 回転を戻しても自己位置は元の位置に戻らず、ひどいときは遠くに動いて行ってしまった。
    * x軸に限らずほかの軸方向でも起こるので、x軸固有の問題ではなさそう。
5. 手で持ち、xyzrpy
    * 4と同じ。

### 加速度、角速度の信用を下げた状態
#### 試験2
| pram | value |
| --- | --- |
| imu_meas_acc_cov | 2.0 |
| acc_cov_output | 10.0 |
| imu_meas_omg_cov | 2.0 |
| gyr_cov_output | 100.0 |
| init_map_size | 1000 |

1. 水平面に接地させる。地面はゴム
    * 静止させ５分おいてみたがドリフトは起きなかった
2. 手で持ち、動かさない。1に対し、細かな揺れの影響を調べる。
    * 影響なし
3. 手で持ち、xyz方向にゆっくり動かす。
    * 影響なし
4. 手で持ち、xyz軸の回転をする。
    * ずれが始まる。
    * デフォルトパラメータに比べ、動きが小さくなった印象
5. 手で持ち、xyzrpyを混ぜて動かす。
    * 4と同じ

#### 試験3
| pram | value |
| --- | --- |
| imu_meas_acc_cov | 2.0 |
| acc_cov_output | 10.0 |
| imu_meas_omg_cov | 2.0 |
| gyr_cov_output | 10.0 |
| init_map_size | 1000 |

1. 水平面に接地させる。地面はゴム
2. 手で持ち、動かさない。1に対し、細かな揺れの影響を調べる。
3. 手で持ち、xyz方向にゆっくり動かす。
4. 手で持ち、xyz軸の回転をする。
    * 明らかに先の実験と比べ回転に対する耐性は上がっている。
    * しかし自己位置のずれは残っている。
    * IMUの軸ではなく、mapの軸方向に対する自己位置のずれが大きい。
5. 手で持ち、xyzrpyを混ぜて動かす。


#### 試験4
データを記録して、同じ動きに対してパラメータを変えて観測する。
記録するデータは、手で回転運動を与えるデータ。  
1. 記録
LiDARドライバを起動する。
```bash
ros2 bag record -o rot_test /unilidar/cloud /unilidar/imu
```

2. 再生
LiDARドライバを起動しない。
```bash
# Point-LIOをパラメータを指定して起動
ros2 launch lidar_bridge real.launch.py lidar:=l1 [param_name:=value ...]

# 記録を再生
ros2 bag play rot_test --clock
```

観察に使用するデータには、各軸方向の回転と回転させたままxy方向に動かすテストをしている。また端子の不具合で一時的にデータが取得できていない期間がある。

##### param.1
| pram | value |
| --- | --- |
| imu_meas_acc_cov | 0.1 |
| acc_cov_output | 1000.0 |
| imu_meas_omg_cov | 0.1 |
| gyr_cov_output | 1000.0 |
| init_map_size | 1000 |

**結果**
回転運動によるずれは観察されなかった。
データが取得できていない期間から復帰したときに大きく自己位置がずれ、遠くに流れていく問題が見られた。


##### param.2
| pram | value |
| --- | --- |
| imu_meas_acc_cov | 2.0 |
| acc_cov_output | 10.0 |
| imu_meas_omg_cov | 2.0 |
| gyr_cov_output | 10.0 |
| init_map_size | 1000 |

**結果**
param.1に比べ、回転運動によるずれが大きかった。
データの復帰後に大きく自己位置がずれる問題はparam.1と同様であった。  

データが途切れていないデータでも試したが、回転運動によるずれの大きさはparam.1のほうが小さかった。

#### 推測
自己位置のずれは、一時的なデータが途切れる問題によるものと推測する。
