#!/bin/bash

python3 rospub_test.py &

# wait to register publishers
sleep 5

topics=(
    "/drone/mavsdk/pteranodon/actuator_control_target"
    "/drone/mavsdk/pteranodon/armed"
    "/drone/mavsdk/pteranodon/attitude_angular_velocity_body"
    "/drone/mavsdk/pteranodon/attitude_euler"
    "/drone/mavsdk/pteranodon/attitude_quaternion"
    "/drone/mavsdk/pteranodon/battery"
    "/drone/mavsdk/pteranodon/camera_attitude_euler"
    "/drone/mavsdk/pteranodon/camera_attitude_quaternion"
    "/drone/mavsdk/pteranodon/capture_info"
    "/drone/mavsdk/pteranodon/connection_state"
    "/drone/mavsdk/pteranodon/distance_sensor"
    "/drone/mavsdk/pteranodon/fixedwing_metrics"
    "/drone/mavsdk/pteranodon/flight_mode"
    "/drone/mavsdk/pteranodon/gps_info"
    "/drone/mavsdk/pteranodon/ground_truth"
    "/drone/mavsdk/pteranodon/heading"
    "/drone/mavsdk/pteranodon/health"
    "/drone/mavsdk/pteranodon/health_all_ok"
    "/drone/mavsdk/pteranodon/home"
    "/drone/mavsdk/pteranodon/imu"
    "/drone/mavsdk/pteranodon/in_air"
    "/drone/mavsdk/pteranodon/information"
    "/drone/mavsdk/pteranodon/landed_state"
    "/drone/mavsdk/pteranodon/mode"
    "/drone/mavsdk/pteranodon/odometry"
    "/drone/mavsdk/pteranodon/position"
    "/drone/mavsdk/pteranodon/position_velocity_ned"
    "/drone/mavsdk/pteranodon/raw_gps"
    "/drone/mavsdk/pteranodon/raw_imu"
    "/drone/mavsdk/pteranodon/rc_status"
    "/drone/mavsdk/pteranodon/scaled_imu"
    "/drone/mavsdk/pteranodon/scaled_pressure"
    "/drone/mavsdk/pteranodon/status"
    "/drone/mavsdk/pteranodon/status_text"
    "/drone/mavsdk/pteranodon/unix_epoch_time"
    "/drone/mavsdk/pteranodon/velocity_ned"
    "/drone/mavsdk/pteranodon/video_stream_info"
    "/drone/mavsdk/pteranodon/vtol_state"
    )

for topic in "${topics[@]}"; do
  ros2 topic list | grep -q "$topic"

  if [ $? -eq 0 ]; then
    echo "'$topic' is being published"
  else
    echo "'$topic' is NOT being published"
  fi
done
