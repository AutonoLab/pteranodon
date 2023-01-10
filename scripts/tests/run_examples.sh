#!/usr/bin/env bash

# Run all the examples in the examples directory
for FILE in examples/base_plugins/*.py;
do
    # run the file if it is not the __init__.py file
    if [ "$FILE" != "examples/__init__.py" ]; then
        python3 $FILE
    fi
done

for FILE in examples/extension_plugins/*.py;
do
    # run the file if it is not the __init__.py file
    if [ "$FILE" != "examples/__init__.py" ]; then
        python3 $FILE
    fi
done
