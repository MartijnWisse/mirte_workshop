#!/usr/bin/env python3
"""
ROS 2 Humble: keyboard publisher (simple, input()-based)

What it does
------------
- Asks the user to type a key or word, then press Enter.
- Publishes that text on the 'key_press' topic (std_msgs/String).
- If the user types exactly 'q' and presses Enter, the node exits.


How students can modify
-----------------------
- Change the topic name ('key_press') to something else.
- Change quit key (currently 'q').
- Map certain inputs to commands (e.g., 'w','a','s','d' to velocities).


If you want more complex changes
--------------------------------
- Give this file to ChatGPT
- Tell it what you want (e.g. use the keyboard to call a ros2 service to open a gripper)
- Ask for explanations of the added / modified code

"""





import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardControlSimple(Node):
    def __init__(self):
        super().__init__('keyboard_control_simple')

        # Publisher: change 'key_press' if you want a different topic name.
        self.pub = self.create_publisher(String, 'key_press', 10)

        # Instructions shown once at startup.
        self.get_logger().info(
            "Type a key or word and press Enter to publish it.\n"
            "Type 'q' and press Enter to quit."
        )

        # Main loop (simple and blocking). No need to spin since we only publish.
        self.loop()

    def loop(self):
        quit_key = 'q'  # <-- students can change this quit condition

        while rclpy.ok():
            try:
                # Prompt; input() blocks until Enter is pressed.
                text = input("> ")
            except (EOFError, KeyboardInterrupt):
                # EOF (Ctrl-D) or Ctrl-C: exit cleanly.
                break

            # Empty lines: skip to keep the topic clean.
            if not text:
                continue

            # Publish the entered text.
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"Published: {repr(text)}")

            # Quit if the user typed the quit key exactly.
            if text == quit_key:
                self.get_logger().info("Quitting...")
                break


def main():
    rclpy.init()
    node = None
    try:
        node = KeyboardControlSimple()
        # No rclpy.spin(node) needed; we do our own input loop.
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
