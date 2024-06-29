# Navigation

This module requires close collaboration with the 'RVIZ' module, to see the result of your work.

## Mapping
We will use a standard package for mapping and navigation; gmapping and amcl. Let's tell the robot to create a map by driving around and storing the lidar data in a map:  
`$ roslaunch mirte_navigation gmapping.launch` 

If this starts without errors, then it is working. Unfortunately, the Mirte Master is not equipped to show any graphics, it can only show text. Therefore, ask the RVIZ team member to add a map display and show the /map topic. You'll see a very incomplete map. Drive around with teleopkey to see the map grow. Don't turn too briskly to create a neat map. If the map is ruined, launch gmapping again.

## Saving the map
If the map looks good, save it *before shutting down gmapping*. In a new terminal, use the following command:  
`$ rosrun map_server map_saver -f /home/mirte/mirte_ws/src/mirte_workshop/maps/default`.  

This command updates the files default.yaml and default.pgm in the directory ~/mirte_ws/src/mirte_workshop/maps. Every time you run the command, these files will be overwritten.Open the .yaml file to see that it contains a reference to the .pgm file, so moving files to different folders must be done with care.

## Localizing with a saved map
If the map is successfully saved, close gmapping and start localization with:  
`$ roslaunch mirte_navigation amcl_demo.launch`  

Again, you need RVIZ to see whether it works. In addition to showing the map, you also want to see the lidar data, the global costmap and the local costmap. The initial position estimate is probably wrong. Make it approximately correct in RVIZ by clicking the "2D Pose Estimate" (green arrow) in the map. This will only work if in the left pane of RVIZ, Displays - Global Options - Fixed Frame is set to "map".

## Using a custom map name
If you wish to use a different map name, use your custom map name in the map_saver command. To use this map for navigation, open the file `~/mirte_ws/src/mirte_navigation/launch/amcl_demo.launch` and change the map name in the line 

    <arg name="map_file" default="$(find mirte_workshop)/maps/default.yaml"/>

## Navigating
A quick and satisfying way to test navigation is to click "2D Nav Goal" (pink arrow) in RVIZ. Pay attention to the terminal from which amcl_demo was launched.  
There is no simple command-line command to set navigation goals. Therefore, we created a python script with a service to set navigation goals.  
`$ rosrun mirte_navigation move_to_server.py` will create the rosservice /move_to. Until the 'markers' team member is ready, the only navigation goals available are 'start' and 'test_location'. From a new terminal, the rosservice can be called with:  
`$ rosservice call /move_to "location: 'start'"`   

It is recommended to check the file move_to_server.py and ask anything that is unclear to ChatGPT. Simply copy the entire code and ask. It will also assist with unexpected errors.

## Tuning navigation parameters
Navigation is endlessly complex. We are now using the nav_core package. There are other packages such as HectorSlam and Slam_toolbox. Even within our current package, there are several planners to choose from. We currently use DWA_local_planner. And then, within this choice, there are many parameters that can be modified.

Try to follow the breadcrumb trail that starts in the file amcl_demo.launch and ends in the loading of several yaml files, each containing many parameter values. The first parameters to change could be:
- min and max velocities
- goal tolerances

It is confusing which parameter value is actually used. Some yaml files contain the same parameters, and one may overwrite the other. To be sure that your edits take effect, use   
`$ rosparam list`, search for your parameter, and check the value with `$ rosparam get /move_base/DWAPlannerROS/xy_goal_tolerance` or any other parameter you want to check.