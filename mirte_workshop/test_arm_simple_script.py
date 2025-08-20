#!/usr/bin/env python3

# ---- Load libraries ----
# standard python libraries
import time

# load the library with ROS functionality for Python
import rclpy
from rclpy.node import Node

# load the specific message formats required for joint commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():

    # ---- Initialize ----
    # start a new ROS node
    rclpy.init()
    node = rclpy.create_node('arm_simple_script')

    # provide on-screen information
    node.get_logger().info('The arm_simple_script has started!')

    # create the publisher, tell it which topic to publish on
    arm_command_publisher = node.create_publisher(JointTrajectory, '/mirte_master_arm_controller/joint_trajectory', 10)

    # give the publisher some time to get ready
    time.sleep(1)



    # ---- Make the first motion ----
    # define the variable type
    trajectory1 = JointTrajectory()
    # use the joint names as used in the Mirte Master
    trajectory1.joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_joint'
    ]
    # define the coordinates
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0, 0.0, 0.0]

    # define how fast to move
    point.time_from_start = Duration(sec=3, nanosec=0)

    # add the position and timing info to the trajectory
    trajectory1.points.append(point)

    # publish the command
    arm_command_publisher.publish(trajectory1)
    # provide on-screen information
    node.get_logger().info('Moving to position 1')




    # give the robot some time to execute the first motion
    time.sleep(3)


    # ---- Make the second motion ----
    trajectory2 = JointTrajectory()
    trajectory2.joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_joint'
    ]
    point2 = JointTrajectoryPoint()
    point2.positions = [1.0, 0.0, 1.0, 1.0]
    point2.time_from_start = Duration(sec=3, nanosec=0)
    trajectory2.points.append(point2)
    arm_command_publisher.publish(trajectory2)
    node.get_logger().info('Moving to position 2')
    time.sleep(3)

    # The end of the function is reached, so it stops

# This is a standard way of programming, making it easier for ROS to handle python functions.
if __name__ == '__main__':
    main()