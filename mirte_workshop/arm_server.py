#!/usr/bin/env python3

# This node is a bit superfluous, because it provides services, which
# will result in message publications for the arm positions, which will, 
# through the arm controller, be translated in yet another set of service requests
# ultimately to set the servo angles. One could set servo angles directly 
# (if the arm controller is turned off).

# The entire code, without any modification, was created with the following ChatGPT 4o prompt (without the # symbols):

# Please create a ROS2 humble node with the following functionality:
# - it provides the following services:
#     /set_arm_home
#     /set_arm_front

# - each of these services are simple Trigger services.
# - each of these publishes a message on the topic /mirte_master_arm_controller/joint_trajectory
# - these are of the type JointTrajectory, details below
# - they represent joint angles for the robot arm, and have the following values per service:

#     /set_arm_home:  [   0,  0,   0,  0]
#     /set_arm_front: [ 0, -1.2,-1.5,1.4]

# Details regarding the JointTrajectory: here is some example code for how to create and publish one of these messages:

#    # ---- Make the first motion ----
#     # define the variable type
#     trajectory1 = JointTrajectory()
#     # use the joint names as used in the Mirte Master
#     trajectory1.joint_names = [
#         'shoulder_pan_joint',
#         'shoulder_lift_joint',
#         'elbow_joint',
#         'wrist_joint'
#     ]
#     # define the coordinates
#     point = JointTrajectoryPoint()
#     point.positions = [0.0, 0.0, 0.0, 0.0]

#     # define how fast to move
#     point.time_from_start = Duration(sec=3, nanosec=0)

#     # add the position and timing info to the trajectory
#     trajectory1.points.append(point)

#     # publish the command
#     arm_command_publisher.publish(trajectory1)
#     # provide on-screen information
#     node.get_logger().info('Moving to position 1')

#     # give the robot some time to execute the first motion
#     time.sleep(3)


# Finally, please keep the structure of the file as simple as possible for easy understanding for beginners.




import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ArmPositionNode(Node):
    def __init__(self):
        super().__init__('arm_position_node')

        # Create the publisher
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10
        )

        # Create the services
        self.create_service(Trigger, '/set_arm_home', self.set_arm_home_callback)
        self.create_service(Trigger, '/set_arm_front', self.set_arm_front_callback)

        self.get_logger().info('Arm Position Node is ready.')

    def create_trajectory(self, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=3, nanosec=0)

        trajectory.points.append(point)
        return trajectory

    def set_arm_home_callback(self, request, response):
        positions = [0.0, 0.0, 0.0, 0.0]
        trajectory = self.create_trajectory(positions)
        self.arm_publisher.publish(trajectory)
        self.get_logger().info('Moving arm to home position.')
        time.sleep(3)
        response.success = True
        response.message = 'Arm moved to home position.'
        return response

    def set_arm_front_callback(self, request, response):
        positions = [0.0, -1.2, -1.5, 1.4]
        trajectory = self.create_trajectory(positions)
        self.arm_publisher.publish(trajectory)
        self.get_logger().info('Moving arm to front position.')
        time.sleep(3)
        response.success = True
        response.message = 'Arm moved to front position.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
