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

i=0
ARGS=" -s "
for arg in "$@"
do 
    CURR=$(($# - i))
    if (( $CURR == 1)) ; then
        ARGS+=" -w $arg"
    elif (( $CURR == 2)) ; then
        ARGS+="$arg:1"
    else
        ARGS+="$arg:1,"
    fi
    let i++
done

./$PX4/Tools/simulation/gazebo/sitl_multiple_run.sh $ARGS &

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
