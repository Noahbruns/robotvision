#!/usr/bin/env python

from ctypes.wintypes import PLARGE_INTEGER
import enum
from math import pi
from quaternions import orientation_from_quaternion, quaternion_from_orientation
from robot import ArcMoveIt
from detect import ArucoDetector
import rospy
import cv2
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, PointStamped
from enum import Enum, auto
from tf.transformations import quaternion_matrix, quaternion_multiply
import numpy as np

# Funktion:
# 0: Greifen
# 5: go to home
# 4: rotate head
# 1: tracking

class Mode(Enum):
    Home = auto()
    Lookout = auto()

    Approach = auto()
    Closing = auto()
    Done = auto()

    RotateHead = auto()

    Tracking = auto()

def move_home(node):
    node.move_taskspace_euler([0, 0.6, 1.2], [1.5707, 0, 0])

def main():
    print("Initializing ROS-node")
    rospy.init_node('handover', anonymous=True)
    target_pub = rospy.Publisher("/target", PoseStamped, queue_size=1)
    rate = rospy.Rate(1)

    transform = tf.TransformListener()

    ic = ArucoDetector()
    node = ArcMoveIt("moveit_py")

    mode = Mode.Home

    while not rospy.is_shutdown():
        marker = ic.best_marker

        # Home Mode
        if mode == Mode.Home:
            move_home(node)
            print("Arrived Home")
            mode = Mode.Lookout
            rospy.sleep(0.5)
            continue

        if mode == Mode.Lookout:
            if marker is None:
                continue

            if marker["id"] == 0:
                print("Found marker 0 -> Grab Object")
                mode = Mode.Approach

            if marker["id"] == 4:
                print("Found marker 2 -> Rotate Head")
                mode = Mode.RotateHead

            if marker["id"] == 1:
                print("Found marker 5 -> Tracking")
                mode = Mode.Tracking

            continue

        # Home Mode
        if marker is not None and mode == Mode.Approach:
            if marker["id"] == 0:
                target = transform.transformPose("r1/world",
                    PoseStamped(
                        header=rospy.Header(frame_id="r1/camera"),
                        pose=marker["pose"]
                    )
                )
                
                # adapt target pose to move infront of it
                target.pose.position.y += 0.4
                target.pose.orientation = node.orientation_from_euler([1.5707, 0, 0])

                # move to new pose
                target_pub.publish(target)
                node.move_taskspace(target)

                rospy.sleep(0.5)
                mode = Mode.Closing
                continue

        # Home Mode
        if marker is not None and mode == Mode.Closing:
            if marker["id"] == 0:
                print("Navigation to Marker ", marker["id"])

                time = transform.getLatestCommonTime("r1/camera", "r1/world")

                #  add transform from cube to camera
                m = TransformStamped()
                m.header.frame_id = "r1/camera"
                m.child_frame_id = "cube"
                m.transform.translation = marker["pose"].position
                m.transform.rotation = marker["pose"].orientation
                m.header.stamp = time
                transform.setTransform(m)

                # find target in cube frame
                t, _ = transform.lookupTransform("r1/camera", "r1/iiwa_link_7", rospy.Time(0))
                target = PoseStamped()
                target.header.frame_id = "cube"
                target.header.stamp = time
                target.pose.position.x = -t[0]
                target.pose.position.y = -t[1]
                target.pose.position.z = -t[2] + 0.03
                target.pose.orientation = node.orientation_from_euler([3.1415, 0, 0])

                # transform to world
                target = transform.transformPose("r1/world", target)

                # move to new pose
                target_pub.publish(target)
                node.move_taskspace_cartesian(target)
                rospy.sleep(0.5)

                mode = Mode.Done
                continue

        if marker is not None and mode == Mode.Done:
            if marker["id"] == 5:
                print("Going Home Now")
                mode = Mode.Home
                continue

        if marker is not None and mode == Mode.RotateHead:
            if marker["id"] == 5:
                print("Going Home Now")
                mode = Mode.Home
                continue

            joints = node.current_joints()

            joints[8] = (joints[8] + pi + pi / 8) % (2 * pi) - pi
            print("Current Joint Positions: ", joints)

            node.move_jointspace(joints)

        if marker is not None and mode == Mode.Tracking:
            if marker["id"] == 5:
                print("Going Home Now")
                mode = Mode.Home
                continue

        rate.sleep()

if __name__ == '__main__':
    main()
