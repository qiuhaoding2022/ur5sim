#!/usr/bin/env python

from __future__ import print_function

from ur5sim.srv import GenObj
import rospy
import sim
###clientID=sim.simxStart('127.0.0.1',20010,True,True,5000,5)
###if clientID!=-1:
##    print ('Connected to remote API server')
##else:
##    print ('Cannot connect to remote API server')
def handle_GenObj(req):
    print("Adding Objection")
    clientID=sim.simxStart('127.0.0.1',20010,True,True,5000,5)
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(clientID,
                               'partsProducer',
                               sim.sim_scripttype_childscript,
                               'generateobject',
                               [],
                               [],
                               [],
                               emptyBuff,
                               sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
    return []
def GenObj_server():
    rospy.init_node('GenObj_server',anonymous=True) 
    s = rospy.Service('GenObj_service', GenObj, handle_GenObj)
    print("Ready to add object")
    rospy.spin()

if __name__ == "__main__":
    GenObj_server()
