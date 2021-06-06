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
from quickstartdemo.srv import GenObj
from math import pi,atan2,degrees,radians
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

print ("OpenCV version:", cv2.__version__)   #Checking OpenCV version, some lines of codes might need to be modified for 4.X version
print ('============ CoppeliaSim setup')
clientID=sim.simxStart('127.0.0.1',20005,True,True,5000,5) #Connect to CoppeliaSim Remote API 
if clientID!=-1:
    print ('Connected to remote API server')

jointhandles={}
for i in range (0,6):
    er,jointhandles[i]=sim.simxGetObjectHandle(clientID,('UR5_joint'+str(i+1)),sim.simx_opmode_blocking) #get joint handle of UR5 in CoppeliaSim
    
print ("============ Starting Moveit setup")  #Following are initializing MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('UR5controller', anonymous=True)   #initialize ros node
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name="manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
scene = moveit_commander.PlanningSceneInterface()
scene.remove_world_object("ground") #Remove ground object just in case
rospy.sleep(2) #a sleep time is needed to add object to the scene
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.y=0
box_pose.pose.position.x=0
box_pose.pose.position.z = -0.5 
box_name = "ground" 
scene.add_box(box_name, box_pose, size=(10, 10, 1)) #here we create a big flat box under the base joint of UR5 to represent the table/ground

print ("============ Opencv setup") 
TASK=-1 # A global variable prevent updating image while the robot arm is on task. Robot arm may be captured by camera and recognized as a object
cv_image=-1 # global variable storing converted image 
class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/objsort/image",Image,self.callback)  #subscribing to camera image topic 

  def callback(self,data): #callback function which converte received image from CoppeliaSim to OpenCV image
    global TASK,cv_image
    try:
        if TASK!=1:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")  
            TASK=1
    except CvBridgeError as e:
      print(e)
      
      
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
    simt((traj[-1].time_from_start)/1.5) #wait some simulation time so that the trajectory can be completed before next executing command is called
    print('execution complete')

# Following code can be called for adding a ceiling boundary to the scene        
##def add_ceil():
##    rospy.sleep(2)
##    box_pose = geometry_msgs.msg.PoseStamped()
##    box_pose.header.frame_id = "world"
##    box_pose.pose.orientation.w = 1.0
##    box_pose.pose.position.y=0
##    box_pose.pose.position.x=0
##    box_pose.pose.position.z = 0.75
##    box_name = "ceil"
##    scene.add_box(box_name, box_pose, size=(10, 10, 0.1))
    
def trajgen(p):  #input p : goal configuration, [roll, pitch, yaw, x, y, z], genereate trajectory using MoveIt
    group.clear_pose_targets()
    group.set_start_state_to_current_state()  
    quaternion = quaternion_from_euler(p[0],p[1], p[2]) #converte euler angle to quanternion because MoveIt use quanternion as configuring position
    pose_goal = geometry_msgs.msg.Pose() #empty pose message
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = p[3]
    pose_goal.position.y = p[4]
    pose_goal.position.z = p[5]
    group.set_pose_target(pose_goal)
    plan = group.plan() #Generate trajectory 
    group.stop
    return plan

def plan_cartesian_path(xscale,yscale,zscale): #input (xscale,yscale,zscale)distance want to move in x,y,z direction
    # generate cartesian path through MoveIt
    group.clear_pose_targets()
    waypoints=[] 
    group.set_start_state_to_current_state()
    pose=geometry_msgs.msg.Pose() #empty pose message
    pose=group.get_current_pose().pose
    quaternion = quaternion_from_euler(0,pi/2, 0) #set orientation for picking up object
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    pose.position.x +=xscale*1
    pose.position.y +=yscale*1
    pose.position.z +=zscale*1
    waypoints.append(copy.deepcopy(pose)) #add pose to waypoint
    (plan, fraction) = group.compute_cartesian_path(        
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.stop
    return plan

def getTOrientation(cnt):  #Use for calculating reference orientation of T shape
  rect=cv2.minAreaRect(cnt) #create a minimum area rectangle around the contour detected 
  x=rect[0][0]          #x position of the center of the rectangle
  y=rect[0][1]          #y position of the cetner of the rectangle
  M=cv2.moments(cnt)    #calculate moment matrix of the contour
  cX = M["m10"] / M["m00"]  #Calculate centroid of image moment, which will be closer to horizontal end of T shape
  cY = M["m01"] / M["m00"]  
  dy=cY-y #use the two centers just calculated as reference, calculate the relative slope 
  dx=cX-x 
  angle=atan2(dy,dx)-pi/2 #converte slope to angle in radian
  return angle

def getIOrientation(cnt):   #use countur fitLine feature to get orientation of I shape
  output=cv2.fitLine(cnt,cv2.DIST_L2,0, 0.01, 0.01)
  dx=output[0]
  dy=output[1]
  angle=atan2(dy,dx)+np.pi/2
  return angle

def simt(dt):#sleep for dt simulation time
    t1=sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]
    while rospy.Duration(secs=(sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]-t1)) < dt:
        pass
    
def suc_pad(action): #calling child script in CoppeliaSim to activate/deactivate suction pad
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
    
def get_relative_position(image, cnt): #Get the center of minAreaRect, and use it as the pick up position of the object
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    rect=cv2.minAreaRect(cnt)
    x=rect[0][0]
    y=rect[0][1]
    x = (x - center_x)/(center_x)
    y = (y - center_y)/(center_y)
    return(x,y)

def conversion(x,y): #Calibration from the returned position from get_relative_position to world position of the object
    x=(x+0.005208)/2.863 
    y=(y+0.005208)/2.863
    return x,y

def getimagecontour(cv_image): #Get contours from the image
    imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  #transfer image to gray scale
    ret, thresh = cv2.threshold(imgray, 127, 255,cv2.THRESH_BINARY_INV) #transfer image to binary
    _,contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #return contours 
    ## uncomment following and comment abouve for OpenCV version 4.X +
    #contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def main():
    global TASK
    ic = image_converter()
    print('Waiting for image to come up')
    while TASK!=1:  #waiting for subscribter to get camera image
        rospy.sleep(1)
    print('Image found')
    
    raw_input("Press enter to continue") #waiting for user input
    
    while TASK==1:
        print('New TASK updated')
        #scene.remove_world_object("ceil")
        ######## Image Processing
        contours=getimagecontour(cv_image)
        printstate=1
        while contours==[]: #If no countours are detected, then call ros service /GenObj_service to add objects to scene
            if printstate==1:
                print('No object found. Calling ros service /GenObj_service ')
                generateobject = rospy.ServiceProxy('GenObj_service', GenObj)
                generateobject()
                generateobject()
                printstate=0
            rospy.sleep(1)
            TASK=0
            contours=getimagecontour(cv_image) #update image
        objectarea=cv2.contourArea(contours[0]) # get area of the contour, used for differentiating shapes
        (x,y)=get_relative_position(cv_image, contours[0])
        x,y=conversion(x,y)
        cpoint=cv2.minAreaRect(contours[0])[0]
        color=cv_image[int(cpoint[1]),int(cpoint[0])] #get the color of the object
        print 'Task Start, starting time is: ',sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]
        pickuppos=[0,pi/2,0,-0.6-y,-x,0.07]  #pick up position in world frame
        plan=trajgen(pickuppos)
        execute_traj(plan)
        rospy.sleep(1.0)
        plan=plan_cartesian_path(0,0,-0.0105)
        execute_traj(plan)
        suc_pad(1)
        simt(rospy.Duration(secs=0.25))
        plan=plan_cartesian_path(0,0,0.1)
        execute_traj(plan)
        if color[2]>100: #Differentiate objects' color base on it's RGB value
            shift=0
        else:
            shift=-0.3
        if objectarea > 700: #Differentiate objects's shaope base on contour area
            angleT=getTOrientation(contours[0])
            print(angleT)
            plan=trajgen([0,pi/2,angleT,0.4+shift,0.4,0.1])
        else:
            angleI=getIOrientation(contours[0])
            print(angleI)
            plan=trajgen([0,pi/2,angleI,0.4+shift,-0.4,0.1])
        execute_traj(plan)
        TASK=0
        suc_pad(0)
        print 'Task Complete, simulation time is: ',sim.simxGetFloatSignal(clientID,'mySimulationTime',sim.simx_opmode_blocking)[1]
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


