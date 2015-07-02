#!/bin/bash

# ROS (For OpenNI2)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
# RSB
sh -c 'echo "deb http://packages.cor-lab.de/ubuntu/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/rsb-latest.list'
wget -q http://packages.cor-lab.de/keys/cor-lab.asc -O- | sudo apt-key add -

# Install missing packages
sudo apt-get update
apt-get install nmap wireshark kate xterm gimp inkscape audacity vlc wine screen vim nano atop iftop htop wavemon subversion git filezilla zip unzip rar unrar p7zip valgrind zsh g++ libboost-all-dev cmake synergy exfat-fuse exfat-utils tcl8.5 tk8.5 expect openssh-server build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libopencv-dev x264 v4l-utils libopenni2-dev openni2-utils openni2-doc konsole ninja-build libeigen3-dev libeigen3-doc libeigen2-dev libeigen2-doc iwidgets4 iwidgets4-doc tclx8.4 tclx8.4-dev ntp
# RSB install
V=0.11
apt-get install sudo apt-get install librsb${V} cl-rsb rsb-tools-cpp${V} rsb-tools-cl${V}
sudo apt-get install spread librsbspread${V}
# Install rsBag
wget -O https://ci.cor-lab.de/job/rsbag-tools-cl-0.12/label=ubuntu_precise_64bit/lastSuccessfulBuild/artifact/build/rsbag > /usr/bin/rsbag


# Install vdemo
cd /tmp
git clone https://code.cor-lab.org/git/vdemo
cd vdemo
mkdir build && cd build
cmake ..
make install
rm -rf /tmp/vdemo

# Add git alias
git config --global alias.co checkout
git config --global alias.br branch
git config --global alias.ci commit
git config --global alias.st status


