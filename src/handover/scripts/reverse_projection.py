import math
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_about_axis, quaternion_from_euler
import os

# Source
# https://math.stackexchange.com/questions/2879056/dimension-of-square-rotated-in-3d-from-projection-on-2d

def solve(pa,pb,pd):

    pb = pb - pa
    pd = pd - pa
    pa = pa - pa

    xb = pb[0]
    yb = pb[1]
    xd = pd[0]
    yd = pd[1]

    v = -(xb * xd + yb * yd)
    u = (xd**2 + yd**2) - (xb**2 + yb**2)

    a = 1
    b = -u
    c = -(v * v)

    #calculate discriminant
    d = b**2 - 4*a*c

    sol1 = (-b - math.sqrt(d)) / (2*a)
    sol2 = (-b + math.sqrt(d)) / (2*a)

    sol = 0
    if (sol1>0):
        sol = sol1
    if (sol2>0):
        sol = sol2

    if (sol==0):
        b = d = 0
    else:
        b = math.sqrt(sol)
        d = abs(v / b)

    ab = np.array([xb, yb, b])
    ad = np.array([xd, yd, d])

    return ab, ad


def corner_to_center(corners):
    M = cv2.moments(corners)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)


def corner_to_area(corners):
    area = cv2.contourArea(corners)
    return area

# estimate 3d normal vector from 3 points of a 2d triangle


def estimate_pose(id, corners, camera_info, diameter):
    # calculate 2 vectors from corners
    v_ab, v_ad = solve(corners[0], corners[1], corners[3])
    center = corner_to_center(corners)

    # calculate vectors
    ab = corners[0] - corners[1]
    dc = corners[3] - corners[2]
    ad = corners[0] - corners[3]
    bc = corners[1] - corners[2]

    ab_length = np.linalg.norm(ab)
    dc_length = np.linalg.norm(dc)
    ad_length = np.linalg.norm(ad)
    bc_length = np.linalg.norm(bc)

    # check sizes of rectangles and adapt angle if necessary
    # if one side of the rectangle is smaller than the other this side is further away from the camera
    if ad_length > bc_length:
        v_ab[2] = -v_ab[2]

    if ab_length > dc_length:
        v_ad[2] = -v_ad[2]

    #print("v_ab: ", v_ab)
    #print("v_ad: ", v_ad)

    # construct Vector A and C to estimate rotation
    # A to C because here the rotation is not affected by projection to 2d
    # TODO improve
    V_ac = corners[0] - corners[2]

    # angle of Vector Vac
    yaw_angle = math.atan2(V_ac[1], V_ac[0]) - math.pi / 4

    # calculate normal vector
    n = np.cross(v_ab, v_ad)
    n = n / np.linalg.norm(n)

    #print("Normal: ", n)

    # calculate yaw angle
    pitch_angle = math.atan2(n[2], n[0])
    roll_angle = math.atan2(n[2], n[1])

    #print("Angels: ", roll_angle, pitch_angle, yaw_angle)
    q = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)

    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    # calculate length of rectangle side
    length = np.linalg.norm(v_ab)

    # reverse projection to get position.
    # Estimate distance by known diameter of aruco
    if camera_info is not None:
        distance = (camera_info.K[0] * diameter) / length
        pose.position.x = distance * \
            (center[0] - camera_info.K[2]) / camera_info.K[0]
        pose.position.y = distance * \
            (center[1] - camera_info.K[5]) / camera_info.K[4]
        pose.position.z = distance

    return pose
