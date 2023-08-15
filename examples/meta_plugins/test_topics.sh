#!/bin/bash

prefix="/drone/mavsdk/pteranodon/"

# python3 manual_publish_test.py &
python3 ros_start_test.py &

pid=$!

# wait to register publishers
sleep 4

topics=(
    "${prefix}actuator_control_target"
    "${prefix}armed"
    "${prefix}attitude_angular_velocity_body"
    "${prefix}attitude_euler"
    "${prefix}attitude_quaternion"
    "${prefix}battery"
    "${prefix}camera_attitude_euler"
    "${prefix}camera_attitude_quaternion"
    "${prefix}capture_info"
    "${prefix}connection_state"
    "${prefix}distance_sensor"
    "${prefix}fixedwing_metrics"
    "${prefix}flight_mode"
    "${prefix}gps_info"
    "${prefix}ground_truth"
    "${prefix}heading"
    "${prefix}health"
    "${prefix}health_all_ok"
    "${prefix}home"
    "${prefix}imu"
    "${prefix}in_air"
    "${prefix}information"
    "${prefix}landed_state"
    "${prefix}mode"
    "${prefix}odometry"
    "${prefix}position"
    "${prefix}position_velocity_ned"
    "${prefix}raw_gps"
    "${prefix}raw_imu"
    "${prefix}rc_status"
    "${prefix}scaled_imu"
    "${prefix}scaled_pressure"
    "${prefix}status"
    "${prefix}status_text"
    "${prefix}unix_epoch_time"
    "${prefix}velocity_ned"
    "${prefix}video_stream_info"
    "${prefix}vtol_state"
    )

for topic in "${topics[@]}"; do
  ros2 topic list | grep -q "$topic"

  if [ $? -eq 0 ]; then
    echo "'$topic' is being published"
  else
    echo "'$topic' is NOT being published"
  fi
done

kill $pid

exit 0
