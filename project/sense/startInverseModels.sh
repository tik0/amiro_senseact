#!/bin/bash
# Start all the inverse models
myPwd=${PWD}
cd ./inverseModelCrop && ./start.sh &
cd ${myPwd}
cd ./inverseModelWeed && ./start.sh &
cd ${myPwd}
cd ./inverseModelFloor && ./start.sh &
cd ${myPwd}
cd ./inverseModelEdge && ./start.sh &
cd ${myPwd}
