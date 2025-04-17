#!/bin/bash

source install/setup.bash && \
ros2 launch gstreamer laptop_launch.py

# sudo sh -c "echo 0 > /sys/bus/usb/devices/1-1/authorized"
# sleep 0.2
# sudo sh -c "echo 1 > /sys/bus/usb/devices/1-1/authorized"

for dev in /sys/bus/usb/devices/[0-9]* /sys/bus/usb/devices/[0-9]-[0-9]*; do
  if [ -f "$dev/product" ] && grep -i -E 'controller|gamepad|joystick' "$dev/product" >/dev/null; then
    if [ -f "$dev/authorized" ]; then
      echo "Resetting controller at $dev"
      echo 0 > "$dev/authorized"
      sleep 0.2
      echo 1 > "$dev/authorized"
      exit 0
    fi
  elif [ -f "$dev/bDeviceClass" ] && [ "$(cat "$dev/bDeviceClass")" = "03" ]; then
    if [ -f "$dev/authorized" ]; then
      echo "Resetting HID device at $dev"
      echo 0 > "$dev/authorized"
      sleep 0.2
      echo 1 > "$dev/authorized"
      exit 0
    fi
  fi
done
