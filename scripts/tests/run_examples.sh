#!/usr/bin/env bash

# find all files that start with 'test_' in the tests/unit/ directory
for FILE in examples/*.py;
do
    # run the file if it is not the __init__.py file
    if [ "$FILE" != "examples/__init__.py" ]; then
        python3 $FILE
    fi
done
