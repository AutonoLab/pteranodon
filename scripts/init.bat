wsl --install -d Ubuntu-20.04

::CALL wsl -e bash -lic ./download_all.sh
CALL wsl echo 'ls'
CALL wsl ../../PX4-Autopilot/Tools/setup/ubuntu.sh

cmd /k