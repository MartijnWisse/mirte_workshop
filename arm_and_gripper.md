# Arm and gripper

Let's make Mirte's arm operational!

## 1. Arm motions from command-line
There are four arm joints. They are controlled by publishing on the topic `/arm/joint_position_controller/command`, e.g.  
```bash
rostopic pub /arm/joint_position_controller/command std_msgs/Float64MultiArray "{data: [0, 0, 0, 0]}"
```
will put the arm straight up.   

Test which number corresponds to which joint, and which rotation direction is positive. It is recommended to make a good drawing.

The robot has two slots for packages on its back. Find the proper angle values for the pick-up arm configuration, so that Mirte Master can grasp the items off of its back in order to place them.

## 2. Arm motions from Python script
To make the robot autonomous, the arm commands should come from code rather than from the command line. Let's try to make the simplest possible Python script that can do this. It is up to you to create a new Python file. We suggest calling it `arm_simple_script.py` and to put it in `~/mirte_ws/src/mirte_workshop/mirte_workshop`.   

Copy the following content to the file:  

```python
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
```

Save the file, move to the script directory and test if it works with:
```bash
cd ~/mirte_ws/src/mirte_workshop/scripts
python3 arm_simple_script.py
```  

Now, you could start editing this file to prepare an entire choreography for the arm! With while-loops you can make it run forever, until stopped with <kbd>Ctrl</kbd>+<kbd>c</kbd>.

## 3. (Optional) Update the ROS2 package
To allow ROS2 to properly manage your new python file, you need to update the ROS2 package. You have done some of these steps before, and they should be repeated each time you add a new python file.

First, open the file `~/mirte_ws/src/mirte_workshop/setup.py`. Find the lines related to the entry points and add one line similar to the others for your new python file.

```python
    entry_points={
        'console_scripts': [
            "arm_simple_script = mirte_workshop.arm_simple_script:main",
        ],
    },
```

After having saved this updated `setup.py` file, re-compile and source the package:

```bash
cd ~/mirte_ws
colcon build --symlink-install --packages-select mirte_workshop
source install/setup.bash
```

Now, ROS2 can find and run the file, which you can test with

```bash
ros2 run mirte_workshop arm_simple_script
```


## 4. Arm motions from service calls
For easy integration with the rest of the robot software, you may want to create ROS services. This will create a small information detour; the service call will effectively publish the same message as we published directly in the code above. But it is still instructive to check out how to create services.

We (well, ChatGPT) prepared an example file for you, `~/mirte_ws/src/mirte_workshop/mirte_workshop/arm_server.py`.  
To start it up, use
```bash
ros2 run mirte_workshop arm_server
```  
It will not actually do anything until a service is requested. In a new terminal,  
```bash
ros2 service call /set_arm_front std_srvs/srv/Trigger
```
will call the service and make the arm move.

This file contains two pre-defined positions. We recommend that you practice adding two more pre-defined positions, for picking up packages off of the robot's back.   
It is recommended to ask ChatGPT for explanations of the code. Simply copy the code and ask, for example "what is the meaning of 'self' in the code above?", or "why are some file names green when I type `ls`?" 

## 5. Gripper servo
The gripper servo motor can be controlled through a 'ROS2 action'. This is an advanced feature for asynchronous tracking of, well, actions, requiring the following command-line command

```bash
ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.2}}"
```

Find out what the maximum and minimum values for the gripper angle are, i.e. fully open and fully closed.

> [!NOTE]  
> Don't let the gripper servo exert too much torque for too long; it will overheat and break. It exerts too much torque when it is trying to reach an angle that is either too far open or too far closed, further than the mechanism allows.

## 6. Gripper service
To simplify controlling the gripper, we created the example node `~/mirte_ws/src/mirte_workshop/mirte_workshop/gripper_server.py`. First, make sure that the 'open' and 'close' values are set to the values that you found out in the previous section. Then test it with   

```bash
ros2 run mirte_workshop gripper_server
```

To use the new services, open a new terminal and try one of the following commands

```bash
ros2 service call /gripper_open std_srvs/srv/Trigger
ros2 service call /gripper_close std_srvs/srv/Trigger
```  

## 7. Integrated service
You now have all the tools you need to make an integrated service. For example, you could create a service that you might call '/deliver_package_1', which, in order:  
- opens the gripper
- brings the arm to the correct configuration to pick up package 1 from its storage slot on its back
- closes the gripper
- brings the arm to the front
- opens the gripper
- brings the arm to the home position

Discuss with your team members what kind of services your application requires.

