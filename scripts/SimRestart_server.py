#!/usr/bin/env python

from __future__ import print_function

from ur5sim.srv import SimRestart
import rospy
import sim
print("============ CoppeliaSim setup")
clientID=sim.simxStart('127.0.0.1',20008,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Cannot connect to remote API server')
def handle_SimRestart(req):
    print("Restarting")
    sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)
    rospy.sleep(10)
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
    return []
def SimRestart_server():
    rospy.init_node('SimRestart_server',anonymous=True) 
    s = rospy.Service('SimRestart_service', SimRestart, handle_SimRestart)
    print("Ready to add object")
    rospy.spin()

if __name__ == "__main__":
    SimRestart_server()
