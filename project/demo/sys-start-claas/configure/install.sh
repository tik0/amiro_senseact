#!/bin/bash
USR=itsowl

# ROS (For OpenNI2)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
# RSB
sh -c 'echo "deb http://packages.cor-lab.de/ubuntu/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/rsb-latest.list'
wget -q http://packages.cor-lab.de/keys/cor-lab.asc -O- | sudo apt-key add -

# Install missing packages
sudo apt-get update
apt-get install nmap wireshark kate xterm gimp inkscape audacity vlc wine screen vim nano iftop htop wavemon subversion git filezilla zip unzip rar unrar p7zip valgrind zsh g++ libboost-all-dev cmake synergy exfat-fuse exfat-utils tcl8.5 tk8.5 expect openssh-server build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev libopencv-dev x264 v4l-utils libopenni2-dev openni2-utils openni2-doc konsole ninja-build libeigen3-dev libeigen3-doc libeigen2-dev libeigen2-doc iwidgets4 iwidgets4-doc tclx8.4 tclx8.4-dev
# RSB install
V=0.12
apt-get install sudo apt-get install librsb${V} cl-rsb rsb-tools-cpp${V} rsb-tools-cl${V}
sudo apt-get install spread librsbspread${V}


# Install vdemo
cd /tmp
git clone https://code.cor-lab.org/git/vdemo
cd vdemo
mkdir build && cd build
cmake ..
make install


# Set proxy set/unset scripts
chown ${USR} /etc/environment
touch /etc/apt/apt.conf.d/01proxy
chown ${USR} /etc/apt/apt.conf.d/01proxy
sudo -H -u ${USR} bash -c "mkdir /home/${USR}/bin"
sudo -H -u ${USR} bash -c "touch /home/${USR}/bin/setProxy.sh && chmod 770 /home/${USR}/bin/setProxy.sh"
sudo -H -u ${USR} bash -c "touch /home/${USR}/bin/unsetProxy.sh && chmod 770 /home/${USR}/bin/unsetProxy.sh"

cat > /home/${USR}/bin/setProxy.sh <<EOF
#!/bin/bash
git config --global http.proxy http://claas\\skiba:Kazuk0099@172.20.20.10:8080
git config --global https.proxy https://claas\\skiba:Kazuk0099@172.20.20.10:8080
echo 'Acquire::http::Proxy "http://claas\skiba:Kazuk0099@172.20.20.10:8080";' > /etc/apt/apt.conf.d/01proxy
echo -e "\
PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games'\n\\
http_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
https_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
ftp_proxy=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
no_proxy='localhost,127.0.0.1,localaddress,.localdomain.com'\n\\
HTTP_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
HTTPS_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
FTP_PROXY=http://claas\\skiba:Kazuk0099@172.20.20.10:8080/\n\\
NO_PROXY='localhost,127.0.0.1,localaddress,.localdomain.com'\\
" > /etc/environment
EOF

cat > /home/${USR}/bin/unsetProxy.sh <<EOF
#!/bin/bash
git config --global --unset http.proxy
git config --global --unset https.proxy
echo '' > /etc/apt/apt.conf.d/01proxy
echo -e "\
PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games'\n\\
no_proxy='localhost,127.0.0.1,localaddress,.localdomain.com'\n\\
NO_PROXY='localhost,127.0.0.1,localaddress,.localdomain.com'\\
" > /etc/environment
EOF

