#!/bin/bash
# arduinoのソースコードをビルドする。
# arduino uno用
# 
# fqbn: Fully Qualified Board Name


~/.local/bin/arduino-cli compile --fqbn arduino:avr:uno $@
