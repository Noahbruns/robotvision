from scipy.optimize import fsolve
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_about_axis
import cv2
import math

# equations for projection
def equations(p, *args):
    x_a, y_a, z_a = 0, 0, p[0]
    x_b, y_b, z_b = p[1], p[2], p[3]
    x_d, y_d, z_d = p[4], p[5], p[6]

    X_b, Y_b = args[0]
    X_d, Y_d = args[1]
    d = args[2]

    return (
        # rules of projection
        x_b - X_b * z_b,
        y_b - Y_b * z_b,
        x_d - X_d * z_d,
        y_d - Y_d * z_d,
        # |AB| = d
        (x_a - x_b) ** 2 + (y_a - y_b) ** 2 + (z_a - z_b) ** 2 - d ** 2,
        # |AD| = d
        (x_a - x_d) ** 2 + (y_a - y_d) ** 2 + (z_a - z_d) ** 2 - d ** 2,
        # AB * AD = 0
        (x_a - x_b) * (x_a - x_d) + (y_a - y_b) * \
        (y_a - y_d) + (z_a - z_b) * (z_a - z_d),
    )

# pa, pb, pd: coordinates of the points
# f: focal length (f_x, f_y)
# d: diameter of the aruco square
def solve(paf, pbf, pdf, d):
    args = (pbf - paf, pdf - paf, d)

    x0 = np.array([
        1.1, # removed a x and y to reduce search space
        0, 0.1, 1.2,
        0.1, 0, 1.0
    ])

    sol = fsolve(equations, x0, args=args)

    if sol[0] < 0:
        # invert the whole solution space
        sol = -sol

    # use distance z to project a
    pa_z = sol[0]
    pa_x = paf[0] * pa_z
    pa_y = paf[1] * pa_z
    a = np.array([pa_x, pa_y, pa_z])

    # use point a to construct points b and d
    b = np.array([pa_x + sol[1], pa_y + sol[2], sol[3]])
    d = np.array([pa_x + sol[4], pa_y + sol[5], sol[6]])

    #print(equations(sol, *args))
    return (a, b, d)


def estimate_pose_perspective(id, corners, camera_info, diameter):
    f = np.array([camera_info.K[0], camera_info.K[4]])
    p = np.array([camera_info.K[2], camera_info.K[5]])

    paf = (np.array(corners[0]) - p) / f
    pbf = (np.array(corners[1]) - p) / f
    # pcf = np.array(corners[2]) - p) / f
    pdf = (np.array(corners[3]) - p) / f

    ab = corners[0] - corners[1]
    dc = corners[3] - corners[2]
    ad = corners[0] - corners[3]
    bc = corners[1] - corners[2]

    ab_length = np.linalg.norm(ab)
    dc_length = np.linalg.norm(dc)
    ad_length = np.linalg.norm(ad)
    bc_length = np.linalg.norm(bc)

    # calculate 2 vectors from corners
    a, b, d = solve(paf, pbf, pdf, diameter)

    # check sizes of rectangles and adapt angle if necessary
    # if one side of the rectangle is smaller than the other this side is further away from the camera
    if ad_length > bc_length:
        b[2] = a[2] + math.abs(b[2] - a[2])
    else:
        b[2] = a[2] - math.abs(b[2] - a[2])

    if ab_length > dc_length:
        d[2] = a[2] + math.abs(d[2] - b[2])
    else:
        d[2] = a[2] - math.abs(d[2] - b[2])
    
    v_ab = a - b
    v_ad = a - d

    # calculate normal vector
    n = np.cross(v_ab, v_ad)
    n = n / np.linalg.norm(n)

    #if id == 0:
        #print(np.round(n, 2))

    # calculate center form v1 and v2
    center = a + (v_ad + v_ad) / 2

    # angle of Vector Vac
    yaw_angle = math.atan2(v_ab[1], v_ab[0])

    # calculate yaw angle
    pitch_angle = math.atan2(n[2], n[0])
    roll_angle = math.atan2(n[2], n[1])

    if id == 0:
        print("Angels: ", roll_angle, pitch_angle, yaw_angle)
    q = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)


    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    pose.position.x = center[0]
    pose.position.y = center[1]
    pose.position.z = center[2]

    return pose
