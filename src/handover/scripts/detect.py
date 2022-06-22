#!/usr/bin/env python3

from os import SEEK_DATA
from re import M
from reverse_projection import solve
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco 
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import math
from tf.transformations import quaternion_about_axis

real_aruco_diameter = 0.05 * 0.7 * 1.8

def corner_to_center(corners):
  M = cv2.moments(corners)
  cx = int(M['m10']/M['m00'])
  cy = int(M['m01']/M['m00'])
  return (cx, cy)

def corner_to_area(corners):
  area = cv2.contourArea(corners)
  return area

# estimate 3d normal vector from 3 points of a 2d triangle
def estimate_pose(corners, camera_info):
  # calculate 2 vectors from corners
  v1, v2 = solve(corners[0][0], corners[0][1], corners[0][3])
  center = corner_to_center(corners)

  # calculate normal vector
  n = np.cross(v1, v2)
  n = n / np.linalg.norm(n)
  n[2] = -n[2]

  # Reference = https://itecnote.com/tecnote/converting-a-direction-vector-to-a-quaternion-rotation/
  realtive = [1, 0, 0]
  S = np.cross(realtive, n)
  angle = np.arccos(np.dot(realtive, n) / (np.linalg.norm(realtive) * np.linalg.norm(n)))
  q = quaternion_about_axis(angle, S)

  pose = Pose()
  pose.orientation.x = q[0]
  pose.orientation.y = q[1]
  pose.orientation.z = q[2]
  pose.orientation.w = q[3]

  length = np.linalg.norm(v1)

  if camera_info is not None:
    distance = (camera_info.K[0] * real_aruco_diameter) / length
    pose.position.x = distance * (center[0] - camera_info.K[2]) / camera_info.K[0]
    pose.position.y = distance * (center[1] - camera_info.K[5]) / camera_info.K[4]
    pose.position.z = distance

  return pose

class ArucoDetector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.pose_pub = rospy.Publisher("/aruco_pose", PoseArray, queue_size=1)
    self.cube_pub = rospy.Publisher("/cube_pose", PoseStamped, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/r1/camera/image_rect", Image, self.callback, queue_size=1)
    self.image_camera_info_sub = rospy.Subscriber("/r1/camera/camera_info", CameraInfo, self.callback_info, queue_size=1)

    self.markers = [None, None, None, None, None, None, None]
    self.best_marker = None
    self.camera_info = None

  def callback_info(self,data):
    self.camera_info = data

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    corners_list, ids_list = self.detect_aruco(cv_image)

    pose_arr = PoseArray()
    pose_arr.header.frame_id = "r1/iiwa_link_7"

    markers = list([None, None, None, None, None, None])
    best_marker = None

    # build map of markers
    for i in range(len(corners_list)):
      markers[ids_list[i][0]] = {
        "id": ids_list[i][0], 
        "center": corner_to_center(corners_list[i]),
        "size": corner_to_area(corners_list[i]),
        "pose": estimate_pose(corners_list[i], self.camera_info),
      }

    # find best marker
    best_marker = None
    for marker in markers:
      if marker is not None and (best_marker is None or marker["size"] > best_marker["size"]):
        best_marker = marker
    
    self.best_marker = best_marker
    self.markers = markers

    #estimate pose of biggest marker
    if self.best_marker is not None:
      self.cube_pub.publish(PoseStamped(header=rospy.Header(frame_id="r1/iiwa_link_7"), pose=self.best_marker["pose"]))
    
    # publish pose hard
    pose_arr.poses = [marker["pose"] for marker in self.markers if marker is not None]
    self.pose_pub.publish(pose_arr)

    markers_img = cv_image
    for i in self.markers:
      if i is not None:
        markers_img = cv2.circle(markers_img, i["center"], 5, (0, 0, 255), -1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(cv_image.shape)

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