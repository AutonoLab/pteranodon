#!/usr/bin/env bash

cd "$(dirname "$0")"
cd ..

pip3 -r install requirements.txt
pip3 -r install requirements-test.txt
pip3 -r install requirements-dev.txt
pip3 -r install requirements-docs.txt
