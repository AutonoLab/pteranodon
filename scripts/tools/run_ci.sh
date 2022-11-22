#!/usr/bin/env bash

python3 -m black --check ./pteranodon
python3 -m flake8 ./pteranodon --count --select=E9,F63,F7,F82,F401,W292,E275,F403 --extend-ignore=W293,W291,E303,E203,W503 --show-source --statistics
python3 -m flake8 ./pteranodon --count --exit-zero --max-complexity=10 --max-line-length=140 --statistics
python3 -m mypy ./pteranodon --no-implicit-optional --show-column-numbers --show-error-codes --disable-error-code import
python3 -m pylint ./pteranodon --disable=C0303,C0114,W0401,R0902,R0904,C0305,W0703,C0200,R0913,R0914,W0107,W1203,W0246
