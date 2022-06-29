#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import copy
import sys
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_about_axis

def all_close(goal, actual, tolerance_d=0.01, tolerance_phi_rad=np.deg2rad(5)):
    # check if goal and actual are within [tolerance_d] meters and [tolerance_phi_rad] radians from each other
    # adapted from http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
    if type(goal) is list:
        for index in range(len(goal)):
            if np.abs(actual[index] - goal[index]) > tolerance_d:
                return False
    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual, tolerance_d, tolerance_phi_rad)
    elif type(goal) is Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = np.linalg.norm([x1-x0, y1-y0, z1-z0])
        cos_phi = np.abs(np.dot([qx0, qy0, qz0, qw0], [qx1, qy1, qz1, qw1]))
        return d <= tolerance_d and cos_phi >= np.cos(tolerance_phi_rad)
    return True


class ArcMoveIt:

    def __init__(self):
        # MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        # move group for "Iiwa" group ("LinAxis" not working atm, thus neither "RoboLab")
        move_group = moveit_commander.MoveGroupCommander("RoboLab")
        move_group.set_planner_id("PRMkConfigDefault")
        self.move_group = move_group
        

        # Demo: some information we can get from the move_group
        # adapted from http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        self.planning_frame = move_group.get_planning_frame()
        rospy.loginfo("== Planning frame: %s" % self.planning_frame)

        eef_link = move_group.get_end_effector_link()
        rospy.loginfo("== End effector link: %s" % eef_link)

        robot = moveit_commander.RobotCommander()
        group_names = robot.get_group_names()
        rospy.loginfo("== Available Planning Groups: %s" % ", ".join(group_names))

    def move_jointspace(self, joint_goal, max_timeout=5):
        # plan and execute
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        timeout = 0
        while timeout < max_timeout and not all_close(joint_goal, self.move_group.get_current_joint_values()):
            rospy.sleep(0.5)
            timeout += 0.5

    def move_taskspace_euler(self, position, orientation, max_timeout=5):
        pose_goal = Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        quaternion = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        
        self.move_taskspace(pose_goal, max_timeout)

    def current_pose(self):
        return self.move_group.get_current_pose().pose

    def current_joints(self):
        return self.move_group.get_current_joint_values()

    def orientation_from_euler(self, euler):
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        orientation = Quaternion()

        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]

        return orientation

    def move_taskspace(self, pose_goal, max_timeout=5):
        self.move_group.set_pose_target(pose_goal)

        # plan and execute
        success, plan, time, errorcode = self.move_group.plan()

        if success:
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            timeout = 0
            while timeout < max_timeout and not all_close(pose_goal, self.move_group.get_current_pose().pose):
                rospy.sleep(0.5)
                timeout += 0.5
        else:
            rospy.logerr("== Planning failed: %s" % errorcode)



    def move_taskspace_cartesian(self, pose_goal, max_timeout=5):
        # adapted from http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        # create a trajectory to follow
        (plan, fraction) = self.move_group.compute_cartesian_path(
            [self.current_pose(), pose_goal.pose],
            0.001, # epsilon
            0 # jump
        )

        # execute the trajectory
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        timeout = 0
        while timeout < max_timeout and not all_close(pose_goal, self.move_group.get_current_pose().pose):
            rospy.sleep(0.5)
            timeout += 0.5


if __name__ == '__main__':
    rospy.init_node("robot", anonymous=True)
    node = ArcMoveIt("moveit_py")

    start_pose = node.move_group.get_current_pose().pose
    print("start_pose: ", start_pose)

    pose_goal = Pose()

    # Pose Position
    pose_goal.position.x = .0
    pose_goal.position.y = .6
    pose_goal.position.z = 1.2

    # Pose Orientation
    quaternion = quaternion_from_euler(1.5707, 0, 0)

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    rospy.loginfo("Go into detection Mode")
    node.move_taskspace(pose_goal)
