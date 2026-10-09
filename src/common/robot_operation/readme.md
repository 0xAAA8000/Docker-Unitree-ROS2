# Robot Operation
Nav2からのTwistを購読し、x軸速度とz軸角速度からPWMとサーボの制御角に変換し、Arduinoに送信。実機車両の操縦を行う。

## 内容物
**config**
- operation.yaml
    * パラメータ設定

**launch**
- operation.launch.py
    * cmd_vel_converterに渡す引数等を