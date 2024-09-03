# RViz

This module must be executed on a computer that has ROS Noetic installed, because RViz requires graphics which Mirte Master cannot display.

## 1. Connecting with ROS on the robot
The robot and the ROS Noetic computer will communicate over TCP/IP, which requires them to know each other's IP address. Hopefully, these are set correctly by the workshop hosts. It can be checked as follows.

1. both of them should have the same ROS Master address, which should be the IP address of the robot with port 11311:
`echo $ROS_MASTER_URI`

2. each should be set their own IP address:
`echo $ROS_IP`

If these settings seem incorrect, ask for assistance.

## 2. Launch RViz

In a terminal in the ROS Noetic computer, type
```console
rviz
```
The following screen will pop up:

![Empty RViz screen](./empty_rviz.png)  

## 3. Show relevant things
Through the 'add' button (encircled in the image above), try to add the following visualizations:

| display type | topic (select in 'Displays' pane) | visualization |
|:-------------|-----------------------------------|---------------|
| Map | /map | shows the map created by gmapping or published by map_server |
| Laserscan | /scan | shows the lidar sensor data |
| TF | Frames 'base_link' and 'map' | shows the coordinate frames |
| Image | /camera/color/image_raw (Transport Hint 'compressed') | shows the front camera image |
| MarkerArray | /stored_points_markers | Only works if `marker_publisher_node.py` is running. Shows stored locations

Some things only work properly if you set the parameter 'Fixed Frame' to 'map', under 'Global Options' (top left of screen). However, initially you should set 'Fixed Frame' to 'base_link'.

Every time you select an item from a drop-down menu, hit the <kbd>Enter</kbd> key.

## 4. Save configuration
To prevent having to set all these things again, save the RViz config with *File* -> *Save Config As*. Navigate to a suitable location on the laptop, enter a name and click the *Save* button. The next time you start RViz, you can specify the configuration:

```console
rviz -d /path/to/your/config/filename.rviz
```
## 5. Help 'navigation' team member(s)

Familiarize yourself with the RViz interface. Learn how to recognize the location of the robot (from the TF frames), enlarge the size of the lidar sensor data, rotate and zoom the map, etcetera. Then, assist the team members working on 'navigation' and 'markers'
