# Arm and gripper

Let's make Mirte's arm operational!

## 1. Arm motions from command-line
There are four arm joints. They are controlled by publishing on the topic `/arm/joint_position_controller/command`, e.g.  
```bash
$ rostopic pub /arm/joint_position_controller/command std_msgs/Float64MultiArray "{data: [0, 0, 0, 0]}"
```
will put the arm straight up.   

Test which number corresponds to which joint, and which rotation direction is positive. It is recommended to make a good drawing.

The robot has two slots for packages on its back. Find the proper angle values for the pick-up arm configuration, so that Mirte Master can grasp the items off of its back in order to place them.

## 2. Arm motions from Python script
To make the robot autonomous, the arm commands should come from code rather than from the command line. Let's try to make the simplest possible Python script that can do this. It is up to you to create a new Python file. We suggest calling it `arm_simple_script.py` and to put it in `~/mirte_ws/src/mirte_workshop/scripts`.   

Copy the following content to the file:  

```python
#!/usr/bin/env python3

# ---- Load libraries ----
# load the library with ROS functionality for Python
import rospy
# load the specific message format required for joint commands
from std_msgs.msg import Float64MultiArray

# ---- Initialize ----
# start a new ROS node
rospy.init_node('arm_simple_script')
# provide on-screen information
rospy.loginfo('The arm_simple_script has started!')
# create the publisher, tell it which topic to publish on
arm_command_publisher = rospy.Publisher('/arm/joint_position_controller/command', Float64MultiArray, queue_size=10)
# give the publisher some time to get ready
rospy.sleep(1)


# ---- Make the first motion ----
# define the variable type
position1 = Float64MultiArray()
# define the variable values
position1.data = [0, 0, 0, 0]
# publish the command
arm_command_publisher.publish(position1)
# provide on-screen information
rospy.loginfo('Moving to position 1')


# give the robot some time to execute the first motion
rospy.sleep(1)


# ---- Make the second motion ----
position2 = Float64MultiArray()
position2.data = [1, 0, 0, 0]
arm_command_publisher.publish(position2)
rospy.loginfo('Moving to position 2')


# The end of the file is reached, so it stops
```

Save the file, move to the script directory and test if it works with:
```bash
$ cd ~/mirte_ws/src/mirte_workshop/scripts
$ python3 arm_simple_script.py
```  

Now, you could start editing this file to prepare an entire choreography for the arm! With while-loops you can make it run forever, until stopped with <kbd>Ctrl</kbd>+<kbd>c</kbd>.

> [!WARNING]  
> rospy catches <kbd>Ctrl</kbd>+<kbd>c</kbd>, so use ``` not rospy.is_shutdown() ``` as while condition.

## 3. Arm motions from service calls
For easy integration with the rest of the robot software, you may want to create ROS services. This will create a small information detour; the service call will effectively publish the same message as we published directly in the code above. But it is still instructive to check out how to create services.

We (well, ChatGPT) prepared an example file for you, `~/mirte_ws/src/mirte_workshop/arm_server.py`.  
To start it up, use
```console
rosrun mirte_workshop arm_server.py
```  
It will not actually do anything until a service is requested. In a new terminal,  
```bash
$ cd ~/mirte
$ rosservice call /set_arm_front "{}"
```
will call the service and make the arm move.

This file contains two pre-defined positions. We recommend that you practice adding two more pre-defined positions, for picking up packages off of the robot's back.   
It is recommended to ask ChatGPT for explanations of the code. Simply copy the code and ask, for example "what is the meaning of 'self' in the code above?", or "why are some file names green when I type `ls`?" 

## 4. Gripper servo
The gripper servo motor can be called directly with the following service call  
```bash
$ rosservice call /mirte/set_servoGripper_servo_angle "angle: 0.0"
```   
> [!NOTE]  
> Similar ROS services exist for the other joints, but the arm controller also uses these same services. Your service call will be overruled by the arm controller.

Find out what the maximum and minimum values for the gripper angle are, i.e. fully open and fully closed.

## 5. Gripper service with success feedback
To know whether a grasp was successful, we created the example node `~/mirte_ws/src/mirte_workshop/gripper_server.py`   
```bash
$ rosrun mirte_workshop gripper_server.py
```
 will start this node. It is instructive to check the code.  
To use the new services, try  
```bash
$ rosservice call /gripper_open "{}"` or `$ rosservice call /gripper_close "{}"
```  
You may have to modify the maximum and minimum values for the gripper angle, they differ from robot to robot.

## 6. Integrated service
You now have all the tools you need to make an integrated service. For example, you could create a service that you might call '/deliver_package_1', which, in order:  
- opens the gripper
- brings the arm to the correct configuration to pick up package 1 from its storage slot on its back
- closes the gripper
- brings the arm to the front
- opens the gripper
- brings the arm to the home position

Discuss with your team members what kind of services your application requires.

