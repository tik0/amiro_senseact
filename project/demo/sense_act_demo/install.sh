#!/bin/bash
#!/
source /opt/poky/1.7.2/environment-setup-cortexa8hf-vfp-neon-poky-linux-gnueabi
cmake_er .
make
./copy.sh
