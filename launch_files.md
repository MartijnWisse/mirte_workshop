# Launch files
Launch files are ideal to start multiple ROS nodes at once. Launch files can include other launch files.

## 1. Command-line launch commands
You have already used the launch command before, when starting keyboard control for driving, with:  
```bash
ros2 launch mirte_teleop teleop_key.launch.py
```

We have also prepared another example launch file for you, which launches the mirte_keyboard.py node that you tested before. Try it out:
```bash
ros2 launch mirte_workshop mirte_example_launch.xml
```

The command structure is `ros2 launch <package_name> <file_name>`. It cleverly knows to find the package folder and to check in the `launch` folder inside that package folder. Verify this by finding the folders and launch files mentioned above, using the folder structure on the left side of the VS Code screen. 

> [!NOTE]  
> Although ROS2 supports Python launch files, as in the teleop_key example, it is preferred to use the lean `.xml` format as in `mirte_example_launch.xml`.

## 2. Create your own launch file
In the folder `~/mirte_ws/src/mirte_workshop/launch`, create a new file and call it `manipulator_launch.xml`.  
Copy the code from `mirte_example_launch.xml` into it, and modify it such that it starts not only `gripper_server.py`, but also `arm_server.py` and `arm_task_server.py`. Then save the file.  
Whenever you add new files, the workpackage must be built again using colcon:  
```bash
cd ~/mirte_ws
colcon build --symlink-install --packages-select mirte_workshop
```
And don't forget to source this in all open terminals (also on other people's computers):  
```bash
cd ~/mirte_ws
source install/setup.bash
```
Finally, test if it works with:
```bash
ros2 launch mirte_workshop manipulator_launch.py
```

> [!NOTE]
> Make sure that nodes are not started twice. Other team members may start nodes from their computer.  

## 3. `ros2 launch` trumps `ros2 run`
`ros2 launch` is at the top of the food chain. It is the most complete and robust way to launch nodes. It is better than the quick command that your team members use:  
```bash
ros2 run mirte_workshop arm_server.py
```
Thus, it is advisable to make nice launch files for them, and eventually one single launch file that launches all required nodes.

By the way, the `ros2 run` command is still better than
```bash
python3 arm_server.py
```
For `ros2 run` and `ros2 launch` to work, you must sometimes tell Linux that your Python files are executable:  
```bash
cd ~/mirte_ws/src/mirte_workshop/<type_here_the_correct_folder>
chmod +x python_file_name.py
``` 

## 4. Include other launch files
Once you have more than one launch file, you may want one launch file to include the other. In such cases, please refer to the relevant tutorials, e.g.:  
https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html 

## 5. Alias
For an even faster start, you can create an 'alias' in Linux. 
Open the file `~/.bashrc` in the editor. Add the following line at the bottom of the file:  

```bash
alias go='ros2 launch mirte_workshop mirte_example_launch.xml'
```

Save the file. All **new** terminals will now execute the `roslaunch` command if you type
```bash
go
```
Existing terminals won't work until you first do  
```bash
source ~/.bashrc
``` 

## 6. Debugging: when the whole ROS2 system breaks
Your launch file started a new ROS2 node. You can see it appear/disappear if you check  
```bash
ros2 node list
```

However, you'll also notice many other nodes that you didn't launch, like `io/telemetrix`. These were started by a background process. You have two ways to control this: invisible and visible.

### 6.1 Invisible ROS2 launch
To stop all mirte's ROS2 nodes, use:
```bash
sudo service mirte-ros stop
```

To start them up again, after waiting about 20 seconds, use:
```bash
sudo service mirte-ros start
```

### 6.2 Visible (informative) ROS2 launch
Stop in the same way:
```bash
sudo service mirte-ros stop
```

Double check that there are no nodes running, other than potentially `rosboard_node`, and `rviz2` on another computer:
```bash
ros2 node list
```

Start it in the visible and informative way:
```bash
ros2 launch mirte_bringup minimal_master.launch.py
```

The terminal will no longer be usable, so open a new terminal to do anything else.
To stop it, use `ctrl`+`c` in the terminal where it started.

> [!NOTE]
> If the basic mirte ROS2 nodes are not running, the robot automatically shuts down after 15 minutes, because it cannot monitor the battery charge.

## 7. Create an integrated system
Discuss with your team members which files need to be started automatically. Some, like keyboard control, are better started separately in their own terminal. Create your complete application!
