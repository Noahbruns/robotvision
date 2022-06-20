#!/usr/bin/python3

import rospy
import arc_ros
import numpy as np


# before you start:
# $catkin build
# $roslaunch arc_gazebo robolab.launch
# $roslaunch arc_ros arc_ros_node.launch

# start example with
# $cd arc/ros/examples
# $python move_robolab.py


rospy.init_node('mover', anonymous=True) 

linear_axis = arc_ros.LinearAxis.LinearAxis()
iiwa = arc_ros.Iiwa.Iiwa()

T_traj = 6

while not rospy.is_shutdown():

  t0 = rospy.Time.now().to_sec()
  q_la = [0.8, 1.2]
  q_iiwa = 0*np.ones((7,1))
  
  linear_axis.move_jointspace(q_la, t0, T_traj,False)
  iiwa.move_jointspace(q_iiwa, t0, T_traj,False)
  rospy.sleep(T_traj)


  t0 = rospy.Time.now().to_sec()
  q_la = [0.3, 0.5]
  q_iiwa = 2*np.ones((7,1))
      
  linear_axis.move_jointspace(q_la, t0, T_traj,False)
  iiwa.move_jointspace(q_iiwa, t0, T_traj,False)
  rospy.sleep(T_traj)