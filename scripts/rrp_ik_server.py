#!/usr/bin/env python3

from __future__ import print_function
import rospy
from math import atan2, acos, sqrt, sin, cos, pi
from rbe500_project.srv import rrpIK


def InverseKinematics(req):
    l2 = 0.425
    l3 = 0.345
    # Get d3
    d3 = 0.05 + 0.45 - 0.11 - req.z
    # Get theta2
    cos_theta2 = (req.x**2 + req.y**2 - l2**2 - l3**2) / (2 * l2 * l3)
    
    if cos_theta2 > 1:
        cos_theta2 = 1

    if cos_theta2 < -1:
        cos_theta2 = -1

    sin_theta2 = sqrt(1 - cos_theta2**2)
    theta2 = atan2(sin_theta2, cos_theta2)

    if theta2 < -pi:
        theta2 += 2 * pi

    if theta2 > pi:
        theta2 -= 2 * pi

    # Get theta1
    theta1 = atan2(req.y, req.x) - atan2(l3 * sin_theta2, l2 + l3 * cos_theta2)

    if theta1 < -pi:
        theta1 += 2 * pi

    if theta1 > pi:
        theta1 -= 2 * pi
        
    return theta1, theta2, d3

def getIK():
    rospy.init_node('IK_Server')
    s = rospy.Service('IK', rrpIK, InverseKinematics)
    print("Ready to get IK")
    rospy.spin()

if __name__ == "__main__":
    getIK()
