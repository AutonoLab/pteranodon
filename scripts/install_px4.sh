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

echo "Installing pip dependencies"
pip install uvloop
pip install mavsdk

echo "Running ubuntu.sh"
sudo bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

sudo reboot

echo "Running Gazebo"
cd ./PX4-Autopilot
sudo make px4_sitl gazebo

echo "Installation Complete"
