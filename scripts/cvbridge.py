#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import atan2, cos, sin, sqrt, pi, copysign,log10,degrees
import time
##def drawAxis(img, p_, q_, colour, scale):
##    p = list(p_)
##    q = list(q_)
##    
##    angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
##    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
##    # Here we lengthen the arrow by a factor of scale
##    q[0] = p[0] - scale * hypotenuse * cos(angle)
##    q[1] = p[1] - scale * hypotenuse * sin(angle)
##    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
##    # create the arrow hooks
##    p[0] = q[0] + 9 * cos(angle + pi / 4)
##    p[1] = q[1] + 9 * sin(angle + pi / 4)
##    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
##    p[0] = q[0] + 9 * cos(angle - pi / 4)
##    p[1] = q[1] + 9 * sin(angle - pi / 4)
##    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
##def getOrientation(pts, img):
##  ## [pca]
##  # Construct a buffer used by the pca analysis
##  sz = len(pts)
##  data_pts = np.empty((sz, 2), dtype=np.float64)
##  for i in range(data_pts.shape[0]):
##    data_pts[i,0] = pts[i,0,0]
##    data_pts[i,1] = pts[i,0,1]
## 
##  # Perform PCA analysis
##  mean = np.empty((0))
##  mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
## 
##  # Store the center of the object
##  cntr = (int(mean[0,0]), int(mean[0,1]))
##  ## [pca]
## 
##  ## [visualization]
##  # Draw the principal components
##  cv2.circle(img, cntr, 3, (255, 0, 255), 2)
##  p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
##  p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
##  drawAxis(img, cntr, p1, (255, 255, 0), 1)
##  drawAxis(img, cntr, p2, (0, 0, 255), 5)
## 
##  angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
##  ## [visualization]
## 
##  # Label with the rotation angle
##  label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
##  print(label)
##  time.sleep(3)
##  #textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
##  #cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
## 
##  return angle
##def getorientation(img):
##  moments = cv2.moments(img)
##  huMoments = cv2.HuMoments(moments)
##  for i in range(0,7):
##    huMoments[i] = -1* copysign(1.0, huMoments[i]) * log10(abs(huMoments[i]))
##  print('')
##  print(huMoments)
##  time.sleep(3)
def getTOrientation(cnt,img):
  rect=cv2.minAreaRect(cnt)
  #print (rect)
  x=int(rect[0][0])
  y=int(rect[0][1])
  cv2.circle(img, (int(rect[0][0]), int(rect[0][1])), 2, (255, 0, 0), -1)
  #x=int(cpoint.pt[0])
  #y=int(cpoint.pt[1])
  M=cv2.moments(cnt)
  #print(M)
  cX = int(M["m10"] / M["m00"])
  cY = int(M["m01"] / M["m00"])
  cv2.circle(img, (cX, cY), 2, (255, 255, 255), -1)
  dy=cY-y
  dx=cX-x
  angle=degrees(atan2(dy,dx))-90
  cv2.imshow('test',img)
  cv2.waitKey(0)
  return angle
def getIOrientation(cnt):
  output=cv2.fitLine(cnt,cv2.DIST_L2,0, 0.01, 0.01)
  dx=output[0]
  dy=output[1]
  angle=atan2(dy,dx)+np.pi/2
  return angle

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/objsort/image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/objsort/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      print(e)

    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold=0.15
    params.filterByCircularity = False
    params.filterByConvexity = False
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(cv_image)
    cpoint=keypoints[0]
    R=cv_image[int(cpoint.pt[1]),int(cpoint.pt[0])]
    #print(R)
    #print('blob')
    #print(x)
    #print(y)
    imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255,cv2.THRESH_BINARY_INV)
    _,contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    angle=getTOrientation(contours[0])
    (x,y)=get_relative_position(cv_image, contours[0])
    print(x,y)
    print(angle)
    #cpoint=cv2.minAreaRect(contours[0])[0]
    #color=cv_image[int(cpoint[1]),int(cpoint[0])]
    #print(color)
    time.sleep(2)
    #cv2.imshow('test',cv_image)
    #cv2.waitKey(0)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
    except CvBridgeError as e:
      print(e)
      
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
  
def get_relative_position(image, cnt):
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


def getTOrientation(cnt):
  rect=cv2.minAreaRect(cnt)
  x=rect[0][0]
  y=rect[0][1]
  M=cv2.moments(cnt)
  cX = (M["m10"] / M["m00"])
  cY = (M["m01"] / M["m00"])
  dy=cY-y
  dx=cX-x
  angle=atan2(dy,dx)-pi/2
  return angle

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
