#!/bin/bash

shopt -s nullglob
for file in /sys/bus/i2c/drivers/ina3221x/7-0040/iio:device0/*; do
    sudo chmod 777 "$file"
done
shopt -u nullglob
