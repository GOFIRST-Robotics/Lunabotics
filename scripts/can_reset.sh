#!/bin/bash
sudo ip link set can0 down
sudo ip link set can0 type can restart-ms 100
sudo ip link set can0 up
candump can0