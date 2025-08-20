#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control')

        # Create service clients
        self.set_arm_package_1 = self.create_client(Trigger, '/set_arm_package_1')
        self.set_arm_package_2 = self.create_client(Trigger, '/set_arm_package_2')
        self.set_arm_home = self.create_client(Trigger, '/set_arm_home')
        self.set_arm_front = self.create_client(Trigger, '/set_arm_front')
        self.gripper_open = self.create_client(Trigger, '/gripper_open')
        self.gripper_close = self.create_client(Trigger, '/gripper_close')

        # Wait for services to be ready
        for client in [
            self.set_arm_package_1,
            self.set_arm_package_2,
            self.set_arm_home,
            self.set_arm_front,
            self.gripper_open,
            self.gripper_close,
        ]:
            client.wait_for_service()

        # Create service servers
        self.create_service(Trigger, '/deliver_package_1', self.handle_deliver_package_1)
        self.create_service(Trigger, '/deliver_package_2', self.handle_deliver_package_2)

        self.get_logger().info("Arm task services /deliver_package_1 and 2 are ready")

    def call_trigger(self, client):
        req = Trigger.Request()
        client.call_async(req)


    def handle_deliver_package_1(self, request, response):
        self.get_logger().info("Step 1")
        self.call_trigger(self.gripper_open)
        self.get_logger().info("Step 2")
        time.sleep(1)

        self.get_logger().info("Step 3")
        self.call_trigger(self.set_arm_package_1)
        self.get_logger().info("Step 4")
        time.sleep(1)

        self.get_logger().info("Step 5")
        self.call_trigger(self.gripper_close)
        self.get_logger().info("Step 6")
        time.sleep(1)

        self.call_trigger(self.set_arm_front)
        time.sleep(1.5)

        self.call_trigger(self.gripper_open)
        time.sleep(1)

        self.call_trigger(self.set_arm_home)

        response.success = True
        response.message = "Package 1 delivered"
        return response

    def handle_deliver_package_2(self, request, response):
        self.call_trigger(self.gripper_open)
        time.sleep(1)

        self.call_trigger(self.set_arm_package_2)
        time.sleep(1)

        self.call_trigger(self.gripper_close)
        time.sleep(1)

        self.call_trigger(self.set_arm_front)
        time.sleep(1.5)

        self.call_trigger(self.gripper_open)
        time.sleep(1)

        self.call_trigger(self.set_arm_home)

        response.success = True
        response.message = "Package 2 delivered"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
