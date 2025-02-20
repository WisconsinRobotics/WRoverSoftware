#!/bin/bash

interface=can0
if [ $# -gt 0 ]; then
    interface=$1
fi

sudo ip link set can0 down

sudo ip link set $interface type can bitrate 500000
sudo ip link set $interface up
sudo ip link set $interface txqueuelen 1000
