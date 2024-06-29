#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import select
import termios
import tty

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_control', anonymous=True)
    pub = rospy.Publisher('key_press', String, queue_size=10)
    
    rospy.loginfo("Press keys to publish them. Press 'q' to quit.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key:
                rospy.loginfo(f"Key pressed: {key}")
                pub.publish(key)
                if key == 'q':
                    rospy.loginfo("Quitting...")
                    break
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
