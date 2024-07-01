# Mirte Master workshop

Welcome to the Mirte Master workshop! Maybe, you have just assembled your own Mirte Master using our open-source designs. More likely, you are participating in a workshop and have been told to visit this page. Anyhow, you are about to give your Mirte Master its first abilities!

## Getting started
### 1. Preparations

Before switching on the robot, make sure that:
- your Mirte Master has a fresh image containing its basic software, 
- the battery is sufficiently well charged,
- the arm is pointing more or less upward.  
*note: if the robot is already on, you cannot move the arm anymore. Don't try too hard, it might break.*

Switch it on, and wait until text appears on the little rear display. As long as the robot doesn't need to drive, keep it connected to the charger.  
**Very important**: The battery will break when over-discharged. Never let the battery percentage go below 10%. As long as ROS is running, it will check battery level and automatically shut down below 10%. Without ROS running, be extremely careful.

### 2. Connecting

The rear display shows a WIFI network name, "mirte-XXXXXX". Connect to it with your laptop.  
**Password**: mirte_mirte  
*note: you will lose internet, unless your laptop has a wired internet connection*

Open a browser on your laptop. Go to the website "http://192.168.42.1:8000"  
**Login username**: mirte  
**Password**: mirte_mirte

You should see the vscode web editor, a powerfull tool to program robots.  
In the bottom left, you can modify "Themes" --> "Color Theme" for better visibility.

### 3. First login

In the vscode web editor, open a new terminal. You will be asked to choose a new password. After changing the password you may have to log in again.  
*note: in Linux, you don't see what you type in the password field, for safety.*

Share the new password with your team members. They can log in simultaneously from their own laptops.

### 4. First Linux and ROS tests

Once you have a terminal, let's refresh your linux skills. For example:  
`$ ls` shows the list of files and folders inside the current folder  
`$ cd folder_name` will change to folder_name  
`$ cd ..` will change one folder up  
`$ python3` will start an interactive python session, exit with `>>> exit()`  
`$ <tab>` will autocomplete your command, very useful to prevent typos  

Let's test that ROS is already running. For example:  
`$ rosnode list` shows all ros nodes that are currently running  
`$ rostopic list` shows all topics that exist  
`$ rostopic echo /topic_name` displays the messages being sent over /topic_name (e.g. /arm/joint_states)  
`$ ctrl-c` stops the last command  
`$ rosservice list` shows all available ros services  

### 5. First robot motions
Driving is controlled through the topic /mobile_base_controller/cmd_vel. Lift up the robot before trying, so that it doesn't drive off the table!   
`$ rostopic pub /mobile_base_controller/cmd_vel <tab> <tab>` change the x value in 0.3  

The robot keeps driving if the message keeps being repeated, with the -r ('rate') option:  
`$ rostopic pub -r 10 /mobile_base_controller/cmd_vel <tab> <tab>` change the x value in 0.3  

Check the messages with `$ rostopic echo` in another terminal 

### 6. First launch of an additional ROS node: driving around!
Driving is easier through keyboard teleoperation. This is available in a ROS node that is not currently running.  
`$ roslaunch mirte_teleop teleopkey.launch` will start keyboard teleoperation.  
Use the x and c keys to tone down linear velocity to 0.3 and angular velocity to 0.6.  
Drive, and in a different terminal check out `rostopic echo /mobile_base_controller/cmd_vel`

## Getting the workshop software on the robot
### 1. Folder structure
Not all required software is on the robot yet. Before fixing that, you need to familiarize yourself with the overall folder structure. Browse folders in the left panel of vscode. The important part of the structure is:

    /home/mirte  
            └── mirte_ws  
                    ├── build  
                    ├── devel  
                    └── src  
                         ├── mirte_navigation    
                         ├── mirte_workshop  
                         ├── mirte_ros_packages  
                         ├── ros_astra_camera  
                         ├── rplidar_ros  

The packages "mirte_navigation" and "mirte_workshop" don't exist yet. We need to get them from github, for which we need an internet connection.  

### 2. Connecting the robot to internet
Make sure that there is a WIFI network that you have control over. For example, use your phone as a hotspot.   
`$ nmcli device wifi connect "SSID" password "PASSWORD"`   (use the correct SSID and PASSWORD) will connect your robot.  

As soon as you give this command, you lose the connection with the robot. To get back in touch:
- Connect your laptop to the same WIFI network
- Find the robot's IP address from your phone hotspot
- Use this IP address in the browser

### 3. Cloning the github repositories 
You are reading this on github. If you scroll up, there is a list of folders and files that together are the package "mirte_workshop". With the green button that says `<> Code`, copy the https address of the repository. Then use the following commands to copy it onto your robot:  
`$ cd ~/mirte_ws/src`  
`$ git clone ...`  paste the copied https address in place of the dots. Use the mouse right click, or use ctrl+shift+v to paste. 

Find the repository mirte_navigation on github and clone that onto the robot as well.


### 4. Compiling new packages
Whenever you git clone a new package onto the robot, it needs to be compiled so that ROS can find and use it. For example:  
`$ cd ~/mirte_ws`  
`$ catkin build mirte_navigation`  
`$ catkin build mirte_workshop`  

Now, in any new terminal, ROS will know how to find the new folders and files. But not in terminals that already exist. To tell them, in each existing terminal you need to type:  
`$ source ~/mirte_ws/devel/setup.bash`  
Alternatively, you can close the terminal(s) and open new ones.

Let's test if it all works with the very underwhelming command  
`$ rosrun mirte_workshop mirte_keyboard.py`  

### 5. Launching mirte_workshop specific configuration
When you turned on the robot, ROS was automatically started. However:  
- This is not exactly the right configuration for the workshop, and  
- It doesn't show screen output, so we don't know what is going on  

`$ sudo service mirte-ros stop`  will stop the invisible ROS instance.  
`$ roslaunch mirte_workshop mirte_workshop.launch`  will start the right one.  

The screen will show which nodes are being started. It will also show error messages, if any. Once launched, you can no longer use this terminal, and ctrl-c will stop ROS. Therefore, open new terminals to run additional commands. 

## Get all the components ready
Here are six workshop modules. The best way to work through them is to assign each module to a different team member.

### [1. Arm and gripper](arm_and_gripper.md)

### [2. Launch files](launch_files.md)

### [3. Keyboard control](keyboard_control.md)

### [4. Navigation](navigation.md)

### [5. RVIZ](rviz.md) *requires laptop/desktop with ROS noetic*

### [6. Markers](markers.md)

## Make a delivery robot
Put all the components together. You are free to create your own scenario. Here is one option:  
- At the press of one key (e.g., '1'), drive to a location and deliver a package
- At the press of another key, drive to a second location and deliver a package
- At the press of a third key, go back to the home position (to collect more packages)