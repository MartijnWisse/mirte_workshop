#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64
from mirte_msgs.srv import SetServoAngle
from mirte_msgs.msg import ServoPosition
import yaml
import os

class GripperController:
    def __init__(self):
        rospy.init_node('gripper_controller')

        self.min_angle = -0.5        # Calibrate these for each robot
        self.max_angle =  0.5

        self.current_angle = None

        self.gripper_position_sub = rospy.Subscriber('/mirte/servos/servoGripper/position', ServoPosition, self.update_gripper_position)
        self.set_servo_angle_service = rospy.ServiceProxy('/mirte/set_servoGripper_servo_angle', SetServoAngle)

        self.gripper_open_service = rospy.Service('/gripper_open', Trigger, self.handle_gripper_open)
        self.gripper_close_service = rospy.Service('/gripper_close', Trigger, self.handle_gripper_close)
        rospy.loginfo('/gripper_open and /gripper_close services available')

    def update_gripper_position(self, msg):
        self.current_angle = msg.angle

    def handle_gripper_open(self, req):
        #rospy.loginfo("Gripper open service called")
        return self.move_gripper(self.max_angle)

    def handle_gripper_close(self, req):
        #rospy.loginfo("Gripper close service called")
        return self.move_gripper(self.min_angle)

    def move_gripper(self, target_angle):
        if self.current_angle is None:
            rospy.logerr("Current gripper position is unknown")
            return TriggerResponse(success=False, message="Current gripper position is unknown")

        # Set the target angle
        self.set_servo_angle_service(target_angle)
        #rospy.loginfo(f"Setting gripper angle to {target_angle}")

        # Monitor the gripper position
        rospy.sleep(0.5)  # Small delay to ensure the command is processed
        previous_angle = self.current_angle

        while not rospy.is_shutdown():
            rospy.sleep(0.1)  # Check every 100 ms
            if abs(self.current_angle - previous_angle) < 0.01:  # Change threshold to detect stop
                break
            previous_angle = self.current_angle

        if target_angle == self.min_angle and self.current_angle > self.min_angle + 0.1:
            # Gripper closed successfully, relax the grip
            #rospy.loginfo("Gripper closed around an object, relaxing the grip")
            self.set_servo_angle_service(self.current_angle)
            return TriggerResponse(success=True, message="Gripper closed successfully")

        if target_angle == self.max_angle and self.current_angle < self.max_angle - 0.1:
            print(target_angle, self.current_angle, self.max_angle)
            # Gripper opened successfully, relax the grip
            #rospy.loginfo("Gripper failed to open fully, relaxing the grip")
            self.set_servo_angle_service(self.current_angle)
            return TriggerResponse(success=False, message="Gripper failed to open fully")

        # If the target angle is reached, but the gripper didn't grip any object or failed to open fully
        if target_angle == self.min_angle:
            #rospy.loginfo("Gripper reached minimal angle without gripping an object")
            return TriggerResponse(success=False, message="Gripper reached minimal angle without gripping an object")
        else:
            #rospy.loginfo("Gripper opened successfully")
            self.set_servo_angle_service(self.current_angle)
            return TriggerResponse(success=True, message="Gripper opened successfully")

if __name__ == "__main__":
    try:
        GripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
