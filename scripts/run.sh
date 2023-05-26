#!/usr/bin/env bash

PX4=third-party/PX4-Autopilot/

# check that the directory is found
if [ ! -d "$PX4" ]; then
    echo "PX4-Autopilot not found"
    exit 1
fi

kill_gazebo () {
  ./scripts/misc/kill_gazebo.sh
}

kill_gazebo

# check to make sure there are 0 or and odd number of args
if (( $# != 0 )) ; then
    if (( $# % 2 != 1 )) ; then
        echo "Invalid number of arguments, expected 0 or an odd number got: $#"
        kill_gazebo
        exit 1
    fi
fi

i=0
ARGS=" -s "
for arg in "$@"
do 
    CURR=$(($# - i))
    if (( i % 2 == 0 )) ; then
        if (( $CURR == 1)) ; then
            ARGS+=" -w $arg"
        elif (( $CURR == 2)) ; then
            ARGS+="$arg:1"
        else
            ARGS+="$arg:1,"
        fi
    fi
    let i++
done

./$PX4/Tools/simulation/gazebo/sitl_multiple_run.sh $ARGS &
sleep 10

i=0
for arg in "$@"
do
    if (( i % 2 == 1 )) ; then
        python3 $arg &
    fi
    let i++
done

echo "Press 'q' to exit"
count=0
while : ; do
    read -n 1 k <&1
    if [[ $k = q ]] ; then
        printf "\nQuitting from the program\n"
        break
    fi
done

kill_gazebo
