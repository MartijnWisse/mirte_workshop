#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand


class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')

        # Action client to control the gripper
        self._action_client = ActionClient(self, GripperCommand, '/mirte_master_gripper_controller/gripper_cmd')

        # Services to trigger open/close
        self.create_service(Trigger, '/gripper_open', self.handle_gripper_open)
        self.create_service(Trigger, '/gripper_close', self.handle_gripper_close)

        self.get_logger().info('Gripper service node is ready.')

    def send_gripper_goal(self, position):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action server not available.')
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position

        self.get_logger().info(f'Sending gripper goal: position = {position}')
        self._action_client.send_goal_async(goal_msg)
        return True


    def handle_gripper_open(self, request, response):
        self.get_logger().info('Received /gripper_open service request')
        success = self.send_gripper_goal(-0.6)
        response.success = success
        response.message = 'Gripper open command sent.' if success else 'Failed to send open command.'
        return response

    def handle_gripper_close(self, request, response):
        self.get_logger().info('Received /gripper_close service request')
        success = self.send_gripper_goal(0.5)
        response.success = success
        response.message = 'Gripper close command sent.' if success else 'Failed to send close command.'
        return response



def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
