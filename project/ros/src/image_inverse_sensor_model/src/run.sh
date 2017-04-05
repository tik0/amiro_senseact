#!/bin/bash

# Pos x: -1.03675  y: 1.00156  z: 3.74036
# Quat qx: -0.99987 qy: -0.00567 qz: -0.01491 qw: -0.00276

echo CAM1
rosrun image_inverse_sensor_model tfInv.py \
_qx:=-0.99987 \
_qy:=-0.00567 \
_qz:=-0.01491 \
_qw:=-0.00276 \
 _x:=-1.03675 \
 _y:=1.00156  \
 _z:=3.74036

 
#  Pos x: 1.24776  y: 1.06103  z: 3.74410
#  Quat qx: -0.99992 qy: -0.00372 qz: 0.01215 qw: 0.00068
     
echo CAM2
rosrun image_inverse_sensor_model tfInv.py \
rosrun image_inverse_sensor_model tfInv.py \
_qx:=-0.99992 \
_qy:=-0.00372 \
_qz:=0.01215  \
_qw:=0.00068  \
 _x:=1.24776  \
 _y:=1.06103  \
 _z:=3.74410

 
# Pos x: 1.35444  y: -1.21880  z: 3.70634
# Quat qx: -0.99912 qy: -0.00805 qz: 0.02940 qw: 0.02870

echo CAM3
rosrun image_inverse_sensor_model tfInv.py \
rosrun image_inverse_sensor_model tfInv.py \
_qx:=-0.99912 \
_qy:=-0.00805 \
_qz:=0.02940  \
_qw:=0.02870  \
 _x:=1.35444  \
 _y:=-1.21880 \
 _z:=3.70634

# Pos x: -0.94061  y: -1.30107  z: 3.67957
# Quat qx: -0.99915 qy: 0.00479 qz: -0.04089 qw: 0.00238

echo CAM4
rosrun image_inverse_sensor_model tfInv.py \
rosrun image_inverse_sensor_model tfInv.py \
_qx:=-0.99915 \
_qy:=0.00479  \
_qz:=-0.04089 \
_qw:=0.00238  \
 _x:=-0.94061 \
 _y:=-1.30107 \
 _z:=3.67957


