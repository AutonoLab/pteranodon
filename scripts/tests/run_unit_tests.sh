#!/usr/bin/env bash

# find all files that start with 'test_' in the tests/unit/ directory
for FILE in tests/unit/test_*.py;
do
    # run the file
    python3 -m pytest --log-cli-level=INFO --full-trace -rP $FILE
done
