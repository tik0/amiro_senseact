#!/usr/bin/env python

import rospy
import time
import sys
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
import tf
from tf import transformations as t

rospy.init_node('tfInf', anonymous=True)
nodeName = rospy.get_name()
print "NodeName:", nodeName
qx = rospy.get_param(nodeName+'/qx', 0)
qy = rospy.get_param(nodeName+'/qy', 0)
qz = rospy.get_param(nodeName+'/qz', 0)
qw = rospy.get_param(nodeName+'/qw', 1)
x = rospy.get_param(nodeName+'/x', 0)
y = rospy.get_param(nodeName+'/y', 0)
z = rospy.get_param(nodeName+'/z', 0)

# main
def main():
  #quaternion = numpy.empty((4, ), dtype=numpy.float64)
  print qx
  quaternion = [qx, qy, qz, qw]
  euler = t.euler_from_quaternion(t.quaternion_inverse(quaternion))
  transform = t.concatenate_matrices(t.translation_matrix([-x, -y, -z]), t.quaternion_matrix(quaternion))
  inversed_transform = t.inverse_matrix(transform)
  print "Inversed tf: "
  print inversed_transform 
  print "HOW TO GO ON with that????? Just get the simple transform which might be wrong"
  print "Trans (m): "
  print [-x, -y, -z]
  print "Rot (rad): "
  print euler

if __name__ == '__main__':
  main()
