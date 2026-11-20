#!/bin/bash
# arduinoのソースコードをビルドする。
#
#   ./scripts/build-arduino.sh firmware/operator                      # Uno 用 (既定)
#   FQBN=arduino:avr:mega ./scripts/build-arduino.sh firmware/operator  # Mega 2560 用
#
# fqbn: Fully Qualified Board Name. 書き込み (upload) も同じ FQBN で行うこと.
# Uno と Mega は書き込みの方式が違うので, 違う FQBN で書き込むと失敗し, 前のプログラムが残る.
#   ~/.local/bin/arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega firmware/operator

FQBN=${FQBN:-arduino:avr:uno}
~/.local/bin/arduino-cli compile --fqbn "$FQBN" "$@"
