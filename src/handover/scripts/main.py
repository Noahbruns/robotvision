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

# Enum of all possible State Machine States
class States(Enum):
    Home = auto()
    Lookout = auto()

    Approach = auto()
    Closing = auto()
    Done = auto()

    RotateHead = auto()

    Tracking = auto()

# State Machine class for better debugging
class StateMachine():
    def __init__(self):
        self.log("StateMachine started")
        self.state = States.Home
        self.change = True

    def setState(self, state):
        self.state = state
        self.change = True
        self.log("State set to: " + str(self.state))
    
    def isChange(self):
        if self.change:
            self.change = False
            return True
        else:
            return False

    def getState(self):
        return self.state

    def isState(self, state):
        return self.state == state

    def log(self, text):
        rospy.loginfo(text)

def main():
    rospy.init_node('handover', anonymous=True)
    target_pub = rospy.Publisher("/target", PoseStamped, queue_size=1)
    rate = rospy.Rate(1)

    # Ros Communicators
    transform = tf.TransformListener()
    ic = ArucoDetector()
    node = ArcMoveIt()

    SM = StateMachine()

    while not rospy.is_shutdown():
        marker = ic.best_marker

        # Home Mode
        if SM.isState(States.Home):
            node.move_taskspace_euler([0, 0.6, 1.2], [1.5707, 1.5707, 0])
            SM.setState(States.Lookout)
            rospy.sleep(0.5)
            continue

        if SM.isState(States.Lookout):
            if marker is None:
                continue

            if marker["id"] == 0:
                rospy.loginfo("Found marker 0 -> Grab Object")
                SM.setState(States.Approach)

            if marker["id"] == 4:
                rospy.loginfo("Found marker 2 -> Rotate Head")
                SM.setState(States.RotateHead)

            if marker["id"] == 1:
                rospy.loginfo("Found marker 5 -> Tracking")
                SM.setState(States.Tracking)

            continue

        # Approach Mode
        if marker is not None and SM.isState(States.Approach):
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
                SM.setState(States.Closing)
                continue

        # Home Mode
        if marker is not None and SM.isState(States.Closing):
            if marker["id"] == 0:
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

                SM.setState(States.Done)
                continue

        if marker is not None and SM.isState(States.Done):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

        if marker is not None and SM.isState(States.RotateHead):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

            joints = node.current_joints()

            joints[8] = (joints[8] + pi + pi / 8) % (2 * pi) - pi

            node.move_jointspace(joints)

        if marker is not None and SM.isState(States.Tracking):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

        rate.sleep()

if __name__ == '__main__':
    main()
