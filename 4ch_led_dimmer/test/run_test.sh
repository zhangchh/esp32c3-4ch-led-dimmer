#!/bin/bash
# 主机端测试：用 mock 的 Arduino API 编译并运行固件，无需开发板。
set -e
cd "$(dirname "$0")"

cp ../4ch_led_dimmer.ino sketch.cpp

echo "== 构建：Arduino-ESP32 3.x =="
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. \
    -DARDUINO_ARCH_ESP32 -DESP_ARDUINO_VERSION_MAJOR=3 \
    -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 \
    sketch.cpp mock_impl.cpp test_main.cpp -o test_v3

echo "== 构建：Arduino-ESP32 2.x（旧的 LEDC 通道 API）=="
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. \
    -DARDUINO_ARCH_ESP32 -DESP_ARDUINO_VERSION_MAJOR=2 \
    -DARDUINO_USB_CDC_ON_BOOT=1 \
    sketch.cpp mock_impl.cpp test_main.cpp -o test_v2

echo
./test_v3
echo
./test_v2 | tail -n 4