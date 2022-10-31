#Download and install PX4.

cd ../..

DIR= /PX4-Autopilot
if [ -d "$DIR" ];
then
    echo "$DIR already exists."
else
    echo "$DIR does not exists, cloning."
	git clone https://github.com/PX4/PX4-Autopilot.git --recursive
fi

./PX4-Autopilot/Tools/setup/ubuntu.sh

sudo reboot