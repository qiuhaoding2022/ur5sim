#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
 
# Detect blobs.
    keypoints = detector.detect(cv_image)
    (x,y)=get_blob_relative_position(cv_image, keypoints[0])
    imgray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255,cv2.THRESH_BINARY_INV)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    a=cv2.contourArea(contours[0])
    b=cv2.contourArea(contours[1])
    c=cv2.contourArea(contours[2])
    im_with_contour=cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
    cv2.imshow("Keypoints", im_with_contour)
    print(a)
    print(b)
    print(c)
    rospy.sleep(1)
    (x,y)=get_blob_relative_position(cv_image, keypoints[0])
    print('blob1')
    print(x)
    print(y)
    (x,y)=get_blob_relative_position(cv_image, keypoints[1])
    print('blob2')
    print(x)
    print(y)
    (x,y)=get_blob_relative_position(cv_image, keypoints[2])
    print('blob2')
    print(x)
    print(y)


    #im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0, 0, 255),  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
# Show blobs
    cv2.waitKey(0)

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
