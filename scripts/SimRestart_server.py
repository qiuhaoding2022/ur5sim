#!/usr/bin/env python

from __future__ import print_function

from ur5sim.srv import SimRestart
import rospy
import sim
#sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)
def handle_SimRestart(req):
    #clientID=sim.simxStart('127.0.0.1',20008,True,True,5000,5)
    if clientID!=-1:
        print("Restarting Simulation,please wait")
        a=sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)
        print 
        rospy.sleep(10)
        sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
        print("Simulation Restarted")
    else:
        print('Cannot connect to remote API server')
    return []
def SimRestart_server():
    rospy.init_node('SimRestart_server',anonymous=True) 
    s = rospy.Service('SimRestart_service', SimRestart, handle_SimRestart)
    print("Restart Server starting")
    rospy.spin()

if __name__ == "__main__":
    SimRestart_server()
