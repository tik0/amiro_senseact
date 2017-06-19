#!/bin/bash

amiroNr=1

./sendOdometryProtoPose -o /amiro${amiroNr}/pose -r 1 > /dev/0 &
./senseSickTim -o /amiro${amiroNr}/laserscan > /dev/0 &
./motorControl -i /amiro${amiroNr}/motor > /dev/0 &
