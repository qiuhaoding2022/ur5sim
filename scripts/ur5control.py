#!/usr/bin/env python
import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sim
import cv2
from sensor_msgs.msg import Image
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from math import pi
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

print '============ CoppeliaSim setup'
#sim.simxFinish(-1) 
clientID=sim.simxStart('127.0.0.1',20005,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
jointhandles={}
for i in range (0,6):
    er,jointhandles[i]=sim.simxGetObjectHandle(clientID,('UR5_joint'+str(i+1)),sim.simx_opmode_blocking)
#sim.simxSynchronous(clientID,True)


print "============ Starting Moveit setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('UR5controller', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name="manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
scene = moveit_commander.PlanningSceneInterface()
scene.remove_world_object("ground")
rospy.sleep(2)
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.y=0
box_pose.pose.position.x=0
box_pose.pose.position.z = -0.5 # slightly above the end effector
box_name = "ground"
scene.add_box(box_name, box_pose, size=(10, 10, 1))

print "============ Opencv setup"
TASK=-1
cv_image=-1
class image_converter:
  def __init__(self):
    #self.image_pub = rospy.Publisher("/objsort/image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/objsort/image",Image,self.callback)

  def callback(self,data):
    global TASK,cv_image
    try:
        if TASK!=1:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            TASK=1
    except CvBridgeError as e:
      print(e)
params = cv2.SimpleBlobDetector_Params()
params.minThreshold=0.15
params.filterByCircularity = False
params.filterByConvexity = False
detector = cv2.SimpleBlobDetector_create(params)
      
def execute_traj(data):
    traj=data.joint_trajectory.points
    for j in range (1,len(traj)):
        targetpos=traj[j].positions
        sim.simxSetJointTargetPosition(clientID,jointhandles[0],targetpos[0]-pi/2,sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID,jointhandles[1],targetpos[1]+pi/2,sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID,jointhandles[2],targetpos[2],sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID,jointhandles[3],targetpos[3]+pi/2,sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID,jointhandles[4],targetpos[4],sim.simx_opmode_streaming)
        sim.simxSetJointTargetPosition(clientID,jointhandles[5],targetpos[5],sim.simx_opmode_oneshot_wait)
    simt((traj[-1].time_from_start)/1.5)
    print('execution complete')
        
def add_ceil():
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.y=0
    box_pose.pose.position.x=0
    box_pose.pose.position.z = 0.75 # slightly above the end effector
    box_name = "ceil"
    scene.add_box(box_name, box_pose, size=(10, 10, 0.1))
    
def trajgen(p):
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    quaternion = quaternion_from_euler(p[0],p[1], p[2])
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = p[3]
    pose_goal.position.y = p[4]
    pose_goal.position.z = p[5]
    group.set_pose_target(pose_goal)
    plan = group.plan()
    group.stop
    return plan

def plan_cartesian_path(xscale,yscale,zscale):
    group.clear_pose_targets()
    waypoints=[]
    group.set_start_state_to_current_state()
    pose=geometry_msgs.msg.Pose()
    pose=group.get_current_pose().pose
    pose.position.x +=xscale*1
    pose.position.y +=yscale*1
    pose.position.z +=zscale*1
    waypoints.append(copy.deepcopy(pose))
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.stop
    return plan

##def plan_cartesian_path2(p):
##    group.clear_pose_targets()
##    waypoints=[]
##    quaternion = quaternion_from_euler(p[0],p[1], p[2])
##    group.set_start_state_to_current_state()
##    pose=geometry_msgs.msg.Pose()
##    pose=group.get_current_pose().pose
##    pose.orientation.x = quaternion[0]
##    pose.orientation.y = quaternion[1]
##    pose.orientation.z = quaternion[2]
##    pose.orientation.w = quaternion[3]
##    pose.position.x = p[3]
##    pose.position.y = p[4]
##    pose.position.z = p[5]
##    waypoints.append(copy.deepcopy(pose))
##    (plan, fraction) = group.compute_cartesian_path(
##                                       waypoints,   # waypoints to follow
##                                       0.01,        # eef_step
##                                       0.0)         # jump_threshold
##    group.stop
##    return plan

def simt(dt):
    t1=sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]
    #secs=sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]-t1
    while rospy.Duration(secs=(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]-t1)) < dt:
        pass
    
def suc_pad(action):
    emptyBuff = bytearray()
    sim.simxCallScriptFunction(clientID,
                               'UR5',
                               sim.sim_scripttype_customizationscript ,
                               'activateSuctionPad',
                               [action],
                               [],
                               [],
                               emptyBuff,
                               sim.simx_opmode_blocking)
    
def get_blob_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (keyPoint.pt[0] - center_x)/(center_x)
    y = (keyPoint.pt[1] - center_y)/(center_y)
    return(x,y)
def conversion(x,y):
    x=(x+0.005208)/2.864583
    y=(y+0.005208)/2.864583
    return x,y

def main():
    global TASK
    ic = image_converter()
    scene.remove_world_object("ceil")
    print('Wait for image to come up')
    while TASK!=1:
        rospy.sleep(1)
    print('Image found')   
    while TASK==1:
        print('New TASK has found')
        ######## Image Processing
        keypoints = detector.detect(cv_image)
        if keypoints==[]:
            print('no object found')
            break
        (x,y)=get_blob_relative_position(cv_image, keypoints[0])
        x,y=conversion(x,y)
        
        imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 127, 255,cv2.THRESH_BINARY_INV)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        objectarea=cv2.contourArea(contours[0])
        print ('Task Start, Starting time is: ')
        print(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1])
        pickuppos=[0,pi/2,0,-0.6,0,0.07]
        plan=trajgen(pickuppos)
        execute_traj(plan)
        rospy.sleep(2)
        plan=plan_cartesian_path(-y,-x,0)
        execute_traj(plan)
        plan=plan_cartesian_path(0,0,-0.011)
        execute_traj(plan)
        suc_pad(1)
    ##    print(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1])
        simt(rospy.Duration(secs=0.25))
    ##    print(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1])
        plan=plan_cartesian_path(0,0,0.1)
        execute_traj(plan)
        #rospy.sleep(1)
        add_ceil()
        if objectarea > 700:
            plan=trajgen([-pi/2,pi/2,pi/2,0.4,0.3,0.1])
        else:
            plan=trajgen([-pi/2,pi/2,pi/2,0.4,-0.3,0.1])
        execute_traj(plan)
        TASK=0
        suc_pad(0)

        print ('Task Complete, simulation time is: ')
        print(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

