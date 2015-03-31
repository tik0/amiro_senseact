#!/bin/bash
# Kill all the inverse models
myPwd=${PWD}
cd ./inverseModelCrop && ./kill.sh &
cd ${myPwd}
cd ./inverseModelWeed && ./kill.sh &
cd ${myPwd}
cd ./inverseModelFloor && ./kill.sh &
cd ${myPwd}
cd ./inverseModelEdge && ./kill.sh &
cd ${myPwd}
