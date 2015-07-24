Installation:

1. Copy the shared objects from lib/x64 or lib/x64_log4cxx to /usr/local/lib 
2. "register" via > sudo ldconfig.
3. lib/x64_log4cxx: sudo apt-get install liblog4cxx10-dev

Network Configuration:

1. Set your nic to dhcp or to LLA (169.254.x.x/255.255.0.0) .
2. Set JumboFrames: sudo ifconfig ethX mtu 9000
3. Increase socket receveive buffer size:  sudo sysctl -w net.core.rmem_max=8388608


