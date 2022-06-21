#!/usr/bin/env python3
import rospy 
import math 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    
    rospy.init_node('set_pose')
    rate = rospy.Rate(20) # 20hz
    
    rospy.wait_for_service('/gazebo/set_model_state')

    state_msg = ModelState()
    state_msg.model_name = 'cube'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    
    x = 0.3
    y = 0.2
    z = 0.8
            
    step = 0.04
    radius = 0.1
    angle = 0
        
    while not rospy.is_shutdown():
        state_msg.pose.position.x = x + radius * math.sin(angle)
        state_msg.pose.position.y = y + radius * math.cos(angle)
        state_msg.pose.position.z = z
        
        pub.publish( state_msg )
            
        angle = (angle + step) % (2 * math.pi)
        
        print(angle)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass