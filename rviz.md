# RVIZ

This module must be executed on a computer that has ROS Noetic installed, because RVIZ requires graphics which Mirte Master cannot display.

## Connecting with ROS on the robot
The robot will be designated as the "ROS Master". This means that the robot will coordinate all the communication with topics, services, etc. We must tell the external computer how to find the ROS Master. This requires two things:  
`$ export ROS_MASTER_URI=http://192.168.xxx.xxx:11311/` with the IP address of the robot (shown on the little screen on the robot, or `$ hostname -I` in a robot terminal)  
`$ export ROS_IP=192.168.yyy.yyy` with the IP address of the computer, obtainable with `$ hostname -I` in the same terminal.  
*note: these parameters must be set in each terminal that you open in the external computer*

Test if all is set correctly with  
`$ rostopic list`  and
`$ rosservice call /mirte/set_servoGripper_servo_angle "angle: -0.2"` 

## Launch RVIZ
The easiest step of all; type `$ rviz`

## Show relevant things
Show:
- axes
- tf (only base_link)
- map (3x)
- MarkerArray
- LaserScan
- Camera (but watch out for large delays)

Set fixed frame to 'map'

## Save configuration
To prevent that you have to set all these things again, save the RVIZ config with ctrl-s (default.rviz) or use "save as". If you don't use default.rviz, provide the config name the next time you start rviz:

`$ rviz -d /path/to/your/config/file.rviz`

