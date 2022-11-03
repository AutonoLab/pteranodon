#!/usr/bin/env bash

#Download and install PX4.
# Run with: sudo bash ./install_px4.sh

cd ../..

DIR= /PX4-Autopilot
if [ -d "$DIR" ];
then
    echo "$DIR already exists."
else
    echo "$DIR does not exists, cloning."
	git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi

echo "Installing Boost................................................................"
sudo apt-get install libboost-all-dev

echo "Installing Gazebo..............................................................."
sudo apt-get install gazebo

echo "Running ubuntu.sh..............................................................."
sudo bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

echo "Installing pip dependencies....................................................."
pip install -r ./pteranodon/requirements.txt


echo "Running Gazebo.................................................................."
cd ./PX4-Autopilot
sudo make px4_sitl gazebo

status=$?
[ $status -eq 0 ] && echo "Installation Complete..........................................................." || echo "Installation Incomplete........................................................."