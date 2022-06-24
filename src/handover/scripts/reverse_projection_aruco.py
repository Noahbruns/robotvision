import math
import cv2
import numpy as np
from geometry_msgs.msg import Pose
import cv2.aruco as aruco
from quaternions import quaternion_from_rvec
from tf.transformations import quaternion_multiply

def estimate_pose_aruco(id, corners, camera_info, diameter):
    # calculate 2 vectors from corners

    mtx = np.array(camera_info.K).reshape((3, 3))
    dist = np.array(camera_info.D)

    rvec, tvec, points = aruco.estimatePoseSingleMarkers(np.array([corners]), diameter, mtx, dist)

    tvec = tvec[0][0]
    rvec = rvec[0][0]

    q = quaternion_from_rvec(rvec)

    # convert to Pose
    pose = Pose()
    pose.position.x = tvec[0]
    pose.position.y = tvec[1]
    pose.position.z = tvec[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose
