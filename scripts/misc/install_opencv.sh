#!/usr/bin/env bash

# INSTALLS
# OPENCV 4.5.2
#    WITH CUDA 6.1 ARCH_BIN SUPPORT
# CUDA 11.3
# CUDNN
# TODO
# make the above variables so that they can be changed

# handle directories
sudo rm -rf ~/opencv/
mkdir ~/opencv

sudo rm -rf ~/cudnn/
mkdir ~/cudnn

# basic updates
sudo apt update -y 
sudo apt upgrade -y

# opencv updates
sudo apt -y install build-essential \
    cmake \
    pkg-config \
    unzip \
    yasm \
    git \
    checkinstall \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavresample-dev
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \  
    libxvidcore-dev \
    x264 \
    libx264-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libfaac-dev \
    libmp3lame-dev \
    libvorbis-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    libdc1394-22 \
    libdc1394-22-dev \
    libxine2-dev \
    libv4l-dev \
    v4l-utils

cd /usr/include/linux &&
sudo ln -s -f ../libv4l1-videodev.h videodev.h -y
cd ~

sudo apt-get -y install libgtk-3-dev \
    python3-dev \
    python3-pip \
    python3-testresources \
    libtbb-dev \
    libatlas-base-dev \
    gfortran \
    libprotobuf-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    libgflags-dev \
    libgphoto2-dev \
    libeigen3-dev \
    libhdf5-dev \
    doxygen \
    ocl-icd-opencl-dev

sudo -H pip3 -y install -U pip numpy

# opencv install and download
cd ~/opencv
wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip
unzip opencv.zip
unzip opencv_contrib.zip
cd ~

# handle CUDA stuff
sudo apt-get -y update
sudo apt-get -y install libcudnn8=8.2.0.53-1+cuda11.3 \
    libcudnn8-dev=8.2.0.53-1+cuda11.3 \
    libcudnn8-samples=8.2.0.53-1+cuda11.3 \
    g++ \
    freeglut3-dev \
    build-essential \
    libx11-dev \
    libxmu-dev \
    libxi-dev \
    libglu1-mesa \
    libglu1-mesa-dev \
    libfreeimage3 \
    libfreeimage-dev

# installs for CUDA
sudo apt-get -y install linux-headers-$(uname -r)
cd ~/cudnn
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get -y update
sudo apt-get -y install cuda, nvidia-gds, zlib1g
cd ~

# install opencv
pip install numpy
cd ~/opencv/opencv-4.5.2
rm -rf build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D WITH_CUBLAS=1 \
	-D WITH_CUDA=ON \
	-D BUILD_opencv_cudacodec=OFF \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D CUDA_ARCH_BIN=6.1 \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib-4.5.2/modules \
	-D ENABLE_CXX11=ON ..
make -j4
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
