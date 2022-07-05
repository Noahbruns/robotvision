#!/usr/bin/env python

from math import pi
import cv2
from enum import auto, Enum
import rospy
import numpy as np
import tf
from detect import ArucoDetector
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Char
from StateMachine import StateMachine
from functions import euler_from_orientation

class States(Enum):
    Home = auto()
    Disabled = auto()
    Lookout = auto()

    Approach = auto()
    Closing = auto()
    Done = auto()

    Tracking = auto()

    Rotate = auto()

speed_limit = 0.8
speed_scale = 1

rot_limit = 0.15
rot_scale = 0.9

def twist(linear, rot):
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

    return action


def main():
    rospy.init_node('handover', anonymous=True)
    tra_pub = rospy.Publisher("/iiwa_cart_vel", Twist, queue_size=1)
    cmd_pub = rospy.Publisher("/iiwa_state_machine_cmd", Char, queue_size=1)

    rate = rospy.Rate(100)

    # Ros Communicators
    transform = tf.TransformListener()
    ic = ArucoDetector()
    SM = StateMachine(States.Home)

    while not rospy.is_shutdown():
        marker = ic.best_marker

        action = twist(np.zeros(3), np.zeros(3))

        if SM.isState(States.Home):
            cmd_pub.publish(ord('i'))
            cmd_pub.publish(ord('4'))
            rospy.sleep(0.1)
            cmd_pub.publish(ord('l'))
            rospy.sleep(0.1)
            cmd_pub.publish(ord('m'))
            rospy.sleep(6)
            cmd_pub.publish(ord('i'))
            SM.setState(States.Disabled)
            continue

        if SM.isState(States.Disabled) and marker is not None:
            if marker["id"] == 0:
                cmd_pub.publish(ord('k'))
                SM.setState(States.Lookout)
                continue

        if SM.isState(States.Lookout) and marker is not None:
            if marker["id"] == 3:
                rospy.loginfo("Found marker 3 -> Grab Object")
                SM.setState(States.Approach)

            if marker["id"] == 1:
                rospy.loginfo("Found marker 1 -> Tracking")
                SM.setState(States.Tracking)

            if marker["id"] == 4:
                rospy.loginfo("Found marker 4 -> Rotating")
                SM.setState(States.Rotate)

        if marker is not None and SM.isState(States.Tracking):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue
            
            if marker["id"] == 1:
                pose = PoseStamped()
                pose.header.frame_id = "detector"
                pose.header.stamp = rospy.Time(0)
                pose.pose = marker["pose"]
                pose = transform.transformPose("/ee_link", pose)

                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.4,
                ])

                (roll, pitch, yaw) = euler_from_orientation(pose.pose.orientation)
                rot = np.array([roll, -pitch, -yaw])

                action = twist(linear, rot)

        if marker is not None and SM.isState(States.Rotate):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue
            
            if marker["id"] == 4:
                pose = PoseStamped()
                pose.header.frame_id = "detector"
                pose.header.stamp = rospy.Time(0)
                pose.pose = marker["pose"]
                pose = transform.transformPose("/ee_link", pose)

                (roll, pitch, yaw) = euler_from_orientation(pose.pose.orientation)
                rot = np.array([roll, -pitch, -yaw])

                action = twist(np.zeros(3), rot)

        # Approach Mode
        if marker is not None and SM.isState(States.Approach):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

            if marker["id"] == 3:
                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.4,
                ])

                if np.sum(np.abs(linear)) < 0.03:
                    linear = np.zeros(3)
                    SM.setState(States.Closing)

                action = twist(linear, np.zeros(3))

        # Closing Mode
        if marker is not None and SM.isState(States.Closing):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue
            
            if marker["id"] == 3:
                pose = PoseStamped()
                pose.header.frame_id = "detector"
                pose.header.stamp = rospy.Time(0)
                pose.pose = marker["pose"]
                pose = transform.transformPose("/ee_link", pose)

                linear = np.array([
                    marker["pose"].position.x,
                    marker["pose"].position.y,
                    marker["pose"].position.z - 0.2,
                ])

                (roll, pitch, yaw) = euler_from_orientation(pose.pose.orientation)
                rot = np.array([roll, -pitch, -yaw])

                if np.sum(np.abs(linear)) < 0.02 and np.sum(np.abs(rot)) < 0.01:
                    linear = np.zeros(3)
                    rot = np.zeros(3)
                    SM.setState(States.Done)

                linear = np.clip(linear, -speed_limit / 2, speed_limit / 2)
                rot = np.clip(rot, -rot_limit / 2, rot_limit / 2)

                action = twist(linear, rot)

            else:
                rospy.loginfo("Marker lost")

        if marker is not None and SM.isState(States.Done):
            if marker["id"] == 5:
                SM.setState(States.Home)
                continue

        
        tra_pub.publish(action)
        rate.sleep()

if __name__ == '__main__':
    main()
