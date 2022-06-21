#!/usr/bin/env python3

from re import M
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco 
import numpy as np

def corner_to_center(corners):
  M = cv2.moments(corners)
  cx = int(M['m10']/M['m00'])
  cy = int(M['m01']/M['m00'])
  return (cx, cy)

def corner_to_area(corners):
  area = cv2.contourArea(corners)
  return area

class ArucoDetector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/r1/camera/image", Image, self.callback, queue_size=1)

    self.markers = []
    self.img_dimensions = (0, 0)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.img_dimensions = cv_image.shape[:2]

    corners_list, ids_list = self.detect_aruco(cv_image)

    if len(corners_list) > 0:
      self.markers = [{
        "id": ids_list[i], 
        "center": corner_to_center(corners_list[i]),
        "size": corner_to_area(corners_list[i]),
      } for i in range(len(ids_list))]
    else:
      self.markers = []

    if ids_list is None:
      self.id_pub.publish(ids_list)
    else:
      ids_str = ''.join(str(e) for e in ids_list)
      self.id_pub.publish(ids_str)

    markers_img = cv_image
    for i in self.markers:
      cv2.circle(markers_img, i["center"], 5, (0, 0, 255), -1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_aruco(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    #print('detected: {}'.format(len(corners)))
    return corners, ids

def main():
  print("Initializing ROS-node")
  rospy.init_node('detect_markers', anonymous=True)
  print("Bring the aruco-ID in front of camera")
  ic = ArucoDetector()
  rospy.spin()

if __name__ == '__main__':
    main()