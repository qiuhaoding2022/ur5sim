#!/usr/bin/env python

import rospy
import sim
print("============ CoppeliaSim setup")
    
def GenObj_server():
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19996,True,True,5000,5)
    print("Adding Objection_serverside")
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print('error')
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
if __name__ == "__main__":
    GenObj_server()
