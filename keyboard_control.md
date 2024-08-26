# Keyboard control

With `teleopkey.launch`, we could easily test the driving functionality. Now, we will create the same easy testing for all our new ROS nodes. 

## 1. Simple keyboard input node
We created a simple file to get started.  
```batch
$ rosrun mirte_workshop mirte_keyboard.py
```
will start the file.   
Presently, it will publish any pressed key in the topic `/key_press`. You can check this in another terminal with
```bash
$ rostopic echo /key_press
```

## 2. Testing topic publishing
The arm could be moved with the following message:  
```bash
$ rostopic pub /arm/joint_position_controller/command std_msgs/Float64MultiArray "{data: [0, 0.5, 0, 0]}"
```   
Instead of having to type this over and over again (even though arrow-up makes this easier), we will publish the message simply by pressing the key '1'.  

In the Python code, the character 'q' is singled out for extra functionality. We will do the same for the character '1'. After the lines

```python
if key == 'q':
    rospy.loginfo("Quitting...")
    break
```

add the lines

```python
elif key == '1':
    rospy.loginfo(f"1 = arm to position 1")
    position1 = Float64MultiArray()
    position1.data = [0,0.5,0,0]
    arm_command_publisher.publish(position1)
```

This is not enough yet. The file needs to be told on which topic to publish. After the line that creates the `/key_press` publisher, add

```python
arm_command_publisher = rospy.Publisher('/arm/joint_position_controller/command', Float64MultiArray, queue_size=1)
```

This command requires that the Python file knows what data type `Float64MultiArray` is, so in the first lines of the file add  

```python
from std_msgs.msg import Float64MultiArray
```

To test your skills, you can add a few more positions. Or you could try to publish on`/mobile_base_controller/cmd_vel`. Or, for example, you could use the key 'y' to increase the value of a variable that you could call `shoulder_angle`, the key 'h' to decrease the value, and then publish `[0, shoulder_angle, 0, 0]`, to move the shoulder anywhere you want.

It is advisable to ask ChatGPT anything that isn't clear to you in the code. Just copy the code and ask. It will also assist when there are unexpected errors.

## 3. Testing service calls
Our file can also be useful to quickly test (new) ROS services. Instead of the command-line service call to move the gripper servo,  
```bash
$ rosservice call /mirte/set_servoGripper_servo_angle "angle: 0.2"
```
we will call the service with the press of a key.  

To tell the file `mirte_keyboard.py` which rosservice to address, add the following line

```python
set_gripper_angle = rospy.ServiceProxy('/mirte/set_servoGripper_servo_angle', SetServoAngle)
```

This requires the following addition in the first few lines of the file:

```python
from mirte_msgs.srv import SetServoAngle
```

The service is now ready to be used when a specific key is pressed, let's use 'g' by adding at the appropriate place:

```python
elif key == 'g':
    rospy.loginfo(f"g = gripper servo to angle 0.2")
    set_gripper_angle(0.2)
```

## 4. Testing command-line commands
Any other command-line command can also be tested through the press of a key. For example, a team member will save maps with:  
```bash
$ rosrun map_server map_saver -f /home/mirte/mirte_ws/src/mirte_workshop/maps/default
```  

We will let the character 'm' execute this command-line command. At the appropriate place, add to `mirte_keyboard.py`:  

```python
elif key == 'm':
    rospy.loginfo(f"m = save map (replaces previous map)")
    os.system(f"rosrun map_server map_saver -f /home/mirte/mirte_ws/src/mirte_workshop/maps/default")
```

## 5. Assist other team members 
Check with your team members which topics, services, or other commands need to be tested and make `mirte_keyboard.py` useful for them. If there are too many key bindings to remember, let the Python file print the available keys and their meanings on screen using the `rospy.loginfo('text goes here')` command. Likely examples to integrate:
- /gripper_close
- /set_arm_home
- /store_current_pose  
> [!NOTE]
> If multiple team members want to run your node simultaneously in their own terminal, they need to launch it as an anonymous node to prevent node name conflicts. Work together with the 'launch file' team member to add the parameter `anon="true"` to the launch file.