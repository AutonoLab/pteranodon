#!/usr/bin/env bash

# Download and install PX4.
# Run with: sudo ./install_px4.sh

# Work as if inside misc folder, no matter where it is called from.
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

cd "$parent_path"
cd ../.. # Change directory to pteranodon root

DIR=./third-party/px4-autopilot/Tools/

if [ -d "$DIR" ];
then
    echo "$DIR already exists."
else
    echo "$DIR does not exists, cloning submodules"
    make submodule-init
    make submodule-update
fi

echo "Running ubuntu.sh..............................................................."
sudo ./third-party/px4-autopilot/Tools/setup/ubuntu.sh --sim_jammy

# ubuntu.sh modifies this file, re-login
source ~/.profile

echo "Installing pip dependencies....................................................."
pip install -r ./requirements.txt

echo "Running Gazebo.................................................................."
cd ./third-party/px4-autopilot/
sudo make px4_sitl_default
sudo make px4_sitl gazebo

# call make one each call for px4_sitl gazebo_*
for TARGET in [gazebo_iris_opt_flow gazebo_solo gazebo_typhoon_h480 gazebo_plane gazebo_plane_catapult gazebo_standard_vtol gazebo_tailsitter gazebo_cloudship]
do
    sudo make px4_sitl $TARGET
done

status=$?
[ $status -eq 0 ] && echo "Installation Complete..........................................................." || echo "Installation Incomplete........................................................."
