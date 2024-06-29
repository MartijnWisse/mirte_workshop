#!/usr/bin/env python3

# This node is a bit superfluous, because it provides services, which
# will result in message publications for the arm positions, which will, 
# through the arm controller, be translated in yet another set of service requests
# ultimately to set the servo angles. One could set servo angles directly 
# (if the arm controller is turned off).

# The entire code, without any modification, was created with the following ChatGPT 4o prompt (without the # symbols):

# Please create a ROS noetic node with the following functionality:
# - it provides the following services:
#     /set_arm_home
#     /set_arm_front

# - each of these services are simple Trigger services.
# - each of these publishes a message on the topic /arm/joint_position_controller/command
# - these are of the type Float64MultiArray
# - they represent joint angles for the robot arm, and have the following values per service:

#     /set_arm_home:  [   0,  0,   0,  0]
#     /set_arm_front: [ 0, -1.2,-1.5,1.4]

# Please keep the structure of the file as simple as possible for easy understanding for beginners.


import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64MultiArray

class ArmControlNode:
    def __init__(self):
        rospy.init_node('arm_control_node')
        
        # Publishers
        self.pub = rospy.Publisher('/arm/joint_position_controller/command', Float64MultiArray, queue_size=10)
        
        # Services
        rospy.Service('/set_arm_home', Trigger, self.set_arm_home)
        rospy.Service('/set_arm_front', Trigger, self.set_arm_front)
        
        rospy.loginfo("Arm control node is ready.")
    
    def set_arm_home(self, req):
        home_position = Float64MultiArray(data=[0, 0, 0, 0])
        self.pub.publish(home_position)
        rospy.loginfo("Arm set to home position.")
        return TriggerResponse(success=True, message="Arm set to home position.")
    
    def set_arm_front(self, req):
        front_position = Float64MultiArray(data=[0, -1.2, -1.5, 1.4])
        self.pub.publish(front_position)
        rospy.loginfo("Arm set to front position.")
        return TriggerResponse(success=True, message="Arm set to front position.")

if __name__ == '__main__':
    try:
        ArmControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
