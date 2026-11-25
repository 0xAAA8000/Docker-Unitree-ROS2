# Robot Operation
Nav2からのTwistを購読し、x軸速度とz軸角速度からPWMとサーボの制御角に変換し、Arduinoに送信。実機車両の操縦を行う。

## 内容物
**設定**
- パラメータは robot_config (`src/common/robot_config/config/` の common.yaml と robots/<ロボット>.yaml の cmd_vel_converter) にまとめてある

**launch**
- operation.launch.py
    * cmd_vel_converterに渡す引数等を