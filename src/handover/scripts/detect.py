#!/usr/bin/env python3

from os import SEEK_DATA
from re import M
from reverse_projection_aruco import estimate_pose_aruco
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco 
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from reverse_projection import corner_to_area, corner_to_center, estimate_pose
import tf

real_aruco_diameter = 0.1 * 0.6

class ArucoDetector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.pose_pub = rospy.Publisher("/aruco_pose", PoseArray, queue_size=1)
    self.cube_pub = rospy.Publisher("/cube_pose", PoseStamped, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=1)
    self.image_camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.callback_info, queue_size=1)

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

    if self.camera_info is None:
      return

    corners_list, ids_list = self.detect_aruco(cv_image)

    pose_arr = PoseArray()
    pose_arr.header.frame_id = "r1/camera"

    markers = list([None, None, None, None, None, None])
    best_marker = None

    # build map of markers
    for i in range(len(corners_list)):
      if ids_list[i][0] < 6:
        markers[ids_list[i][0]] = {
          "id": ids_list[i][0], 
          "center": corner_to_center(corners_list[i]),
          "size": corner_to_area(corners_list[i]),
          "pose": estimate_pose_aruco(ids_list[i][0], corners_list[i][0], self.camera_info, real_aruco_diameter),
        }

    # find best marker
    best_marker = None
    for marker in markers:
      if marker is not None and (best_marker is None or marker["size"] > best_marker["size"]):
        best_marker = marker
    
    self.best_marker = best_marker
    self.markers = markers

    #estimate pose of biggest marker
    if self.markers[0] is not None:
      self.cube_pub.publish(PoseStamped(header=rospy.Header(frame_id="r1/camera"), pose=self.markers[0]["pose"]))
    
    # publish pose array
    pose_arr.poses = [marker["pose"] for marker in self.markers if marker is not None]
    self.pose_pub.publish(pose_arr)

    markers_img = cv_image
    for i in self.markers:
      if i is not None:
        markers_img = cv2.circle(markers_img, i["center"], 5, (0, 0, 255), -1)

    markers_img = aruco.drawDetectedMarkers(markers_img, corners_list, ids_list)  # detect the sruco markers and display its aruco id.

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