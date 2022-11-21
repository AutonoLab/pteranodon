#!/usr/bin/env bash

# find all files that start with 'test_' in the tests/integration/ directory
for FILE in tests/integration/test_*.py;
do
    # run the file
    python3 -m pytest $FILE
done
