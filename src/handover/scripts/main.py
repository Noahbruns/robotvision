#!/usr/bin/env python

from robot import ArcMoveIt
from detect import ArucoDetector
import rospy
import cv2

def move_home(node):
    node.move_taskspace_euler([0, 0.6, 1.2], [1.5707, 0, 0])

def main():
    print("Initializing ROS-node")
    rospy.init_node('handover', anonymous=True)
    rate = rospy.Rate(0.2)

    ic = ArucoDetector()
    node = ArcMoveIt("moveit_py")

    move_home(node)

    while not rospy.is_shutdown():
        markers = ic.markers

        # get biggest marker
        if len(markers) > 0:
            markers.sort(key=lambda x: x["size"])
            marker = markers[-1]
            print("Largest Marker: {}".format(marker))

            print(marker["center"], ic.img_dimensions)

            # calculate offset from image center
            marker_offset = (marker["center"][0] - ic.img_dimensions[0]/2, marker["center"][1] - ic.img_dimensions[1]/2)
            print("Marker offset: {}".format(marker_offset))

            pose = node.current_pose()

            print(pose)

            # callculate new pose
            pose.position.x += marker_offset[0] / 500.
            #pose.position.y += marker_offset[1] / 500.

            print("New pose: {}".format(pose))

            # move to new pose
            node.move_taskspace(pose)


        rate.sleep()

if __name__ == '__main__':
    main()
