from tf.transformations import quaternion_from_matrix
import cv2
import numpy as np
from geometry_msgs.msg import Quaternion

def quaternion_from_rvec(rvec):
    # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

    # convert the matrix to a quaternion
    return quaternion_from_matrix(rotation_matrix)

def quaternion_from_orientation(o):
    return np.array([o.x, o.y, o.z, o.w])

def orientation_from_quaternion(q):
    return Quaternion(q[0], q[1], q[2], q[3])