#!/usr/bin/env python

from math import pi
import cv2
import enum
from enum import auto, Enum
import rospy
import numpy as np
import tf
from detect import ArucoDetector
from geometry_msgs.msg import Twist
from std_msgs.msg import Char
from tf.transformations import euler_from_quaternion

class States(Enum):
    Home = auto()
    Lookout = auto()

    Approach = auto()
    Closing = auto()
    Done = auto()

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

def euler_from_pose(pose):
    q = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]

    return euler_from_quaternion(q)


def main():
    rospy.init_node('handover', anonymous=True)
    tra_pub = rospy.Publisher("/iiwa_cart_vel", Twist, queue_size=1)
    cmd_pub = rospy.Publisher("/iiwa_state_machine_cmd", Char, queue_size=1)

    rate = rospy.Rate(100)

    # Ros Communicators
    transform = tf.TransformListener()
    ic = ArucoDetector()
    SM = StateMachine()

    speed_limit = 0.06
    speed_scale = 0.3

    rot_limit = 0.05
    rot_scale = 0.3

    def vel_ctl(linear, rot):
        action = Twist()

        linear = linear * speed_scale
        linear = np.clip(linear, -speed_limit, speed_limit)        
        action.linear.x = linear[0]
        action.linear.y = linear[1]
        action.linear.z = linear[2]

        rot = rot * rot_scale
        rot = np.clip(rot, -rot_limit, rot_limit)
        action.angular.x = rot[0]
        action.angular.y = rot[1]
        action.angular.z = rot[2]

        tra_pub.publish(action)
        rate.sleep()

    while not rospy.is_shutdown():
        marker = ic.best_marker

        linear = np.zeros(3)
        rot = np.zeros(3)

        if SM.isState(States.Home):
            cmd_pub.publish(ord('i'))
            rospy.sleep(0.1)
            cmd_pub.publish(ord('l'))
            rospy.sleep(0.1)
            cmd_pub.publish(ord('m'))
            rospy.sleep(6)
            cmd_pub.publish(ord('k'))
            SM.setState(States.Lookout)
            continue

        if SM.isState(States.Lookout) and marker is not None:
            if marker["id"] == 0:
                rospy.loginfo("Found marker 0 -> Grab Object")
                SM.setState(States.Approach)

            if marker["id"] == 1:
                rospy.loginfo("Found marker 1 -> Tracking")
                SM.setState(States.Tracking)

        if marker is not None and SM.isState(States.Tracking):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue
            
            if marker["id"] == 1:
                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.3,
                ])

                (roll, pitch, yaw) = euler_from_pose(marker["pose"])

                #roll = ((roll + 2 * pi) % 2 * pi) - pi
                #print(roll)

                #rot[0] = roll
                #rot[1] = pitch

        # Approach Mode
        if marker is not None and SM.isState(States.Approach):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

            if marker["id"] == 0:
                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.4,
                ])

                if np.sum(linear) < 0.03:
                    linear = np.zeros(3)
                    SM.setState(States.Closing)

        # Approach Mode
        if marker is not None and SM.isState(States.Closing):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue
            
            if marker["id"] == 0:
                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.2,
                ])

                if np.sum(linear) < 0.03:
                    linear = np.zeros(3)
                    SM.setState(States.Done)

                linear = np.clip(linear, -speed_limit / 2, speed_limit / 2)

        if marker is not None and SM.isState(States.Done):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

        vel_ctl(linear, rot)
        rate.sleep()

if __name__ == '__main__':
    main()
