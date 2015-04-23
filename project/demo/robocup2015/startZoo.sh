#!/bin/bash
./killZoo.sh
spread -c amirospread &
sleep 5

./choreoStateMachine -s --pos $1 
#./choreoStateMachine -s --pos `ifconfig | grep  -o -e "192.168.2.[0-9]*\ " | head -n 1 | sed "s#192.168.2.##g"`
