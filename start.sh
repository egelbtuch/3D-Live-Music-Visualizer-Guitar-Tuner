#!/usr/bin/env bash
sleep 2
sudo python3 /home/pi/rpi-rgb-led-matrix/bindings/python/samples/Visual.py\
 --led-rows=64 --led-cols=64\
 --led-gpio-mapping=adafruit-hat --led-slowdown-gpio=2\
 --led-pwm-lsb-nanoseconds=50 --led-pwm-bits=3 
