#!/usr/bin/env python

from ctypes.wintypes import PLARGE_INTEGER
import enum
from robot import ArcMoveIt
from detect import ArucoDetector
import rospy
import cv2
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from enum import Enum
from tf.transformations import quaternion_from_euler

class Mode(Enum):
    Home = 0
    Approach = 1
    Grab = 2

def move_home(node):
    node.move_taskspace_euler([0, 0.6, 1.2], [1.5707, 0, 0])

def main():
    print("Initializing ROS-node")
    rospy.init_node('handover', anonymous=True)
    rate = rospy.Rate(1)

    transform = tf.TransformListener()

    ic = ArucoDetector()
    node = ArcMoveIt("moveit_py")

    move_home(node)
    mode = Mode.Home
    print("Mode set to Home")

    while not rospy.is_shutdown():
        markers = ic.markers

        # Home Mode
        if len(markers) > 0 and mode == Mode.Home:
            marker = ic.best_marker
            
            if marker is not None:
                print("Marker found: ", marker["id"])
                if marker["id"] == 0:
                    print("Navigation to Marker ", marker["id"])

                    target = transform.transformPose("r1/world",
                        PoseStamped(
                            header=rospy.Header(frame_id="r1/iiwa_link_7"), 
                            pose=marker["pose"]
                        )
                    )
                    
                    # adapt target pose to move infront of it
                    target.pose.position.y += 0.3
                    target.pose.orientation = node.orientation_from_euler([1.5707, 0, 0])

                    print("Target Pose: ", target)

                    # move to new pose
                    node.move_taskspace(target)

                    mode = Mode.Approach
                    print("Mode set to Target")

        # Home Mode
        if len(markers) > 0 and mode == Mode.Approach:
            marker = ic.best_marker
            
            if marker is not None:
                print("Marker found: ", marker["id"])
                if marker["id"] == 0:
                    print("Navigation to Marker ", marker["id"])

                    target = transform.transformPose("r1/world",
                        PoseStamped(
                            header=rospy.Header(frame_id="r1/iiwa_link_7"), 
                            pose=marker["pose"]
                        )
                    )
                    
                    # adapt target pose to move infront of it and ro
                    target.pose.position.y += 0.05
                    target.pose.orientation = node.orientation_from_euler([1.5707, 0, 0])
                    #arr = [target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w]
                    #test = arr * quaternion_from_euler(-1.5707, 0, 0)
                    #target.pose.orientation.x = test[0]
                    #target.pose.orientation.y = test[1]
                    #target.pose.orientation.z = test[2]
                    #target.pose.orientation.w = test[3]

                    print("Target Pose: ", target)

                    # move to new pose
                    node.move_taskspace_cartesian(target)

                    mode = Mode.Grab
                    print("Mode set to Grab")

        rate.sleep()

if __name__ == '__main__':
    main()
