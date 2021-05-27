#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from quickstartdemo.srv import GenObj

def genereate_object_client():
    rospy.wait_for_service('GenObj_service')
    try:
        generateobject = rospy.ServiceProxy('GenObj_service', GenObj)
        generateobject()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Adding Objection_clidentside")
    genereate_object_client()
