#!/usr/bin/env bash

# handles command line installation of tools which pteranodon users will use or are likely to use
sudo apt-get install -y git-all
sudo apt install -y libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
sudo apt install -y python3-gi python3-gi-cairo gir1.2-gtk-3.0

# installations for the extensions modules
sudo apt install -y libopencv-dev

# install specific python package for PX4
pip3 install empy==3.*
pip3 install jinja2==3.*
