#!/usr/bin/env bash

for FILE in requirements*.txt;
do
    pip3 install -r $FILE
done
