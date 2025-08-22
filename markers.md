# Markers

In this module, we will give names ('labels') to specific locations on the map.  
This module assumes that you have installed the package `mirte_location_markers`.  
It also assumes that the `mirte_navigation` is running properly.

## 1. Poses
In a terminal on the robot, run the following command:
```bash
ros2 run mirte_location_markers marker_publisher.py
```  

Collaborate with the RViz2 team to visualize the markers as a 'MarkerArray' in RViz2. You should see two green dots on the map, one called 'start' and the other called 'test_location'

The locations of these dots are defined in the file `~/mirte_ws/src/mirte_location_markers/locations/stored_poses.yaml`. Open the file and analyse its content. 

- Orientations are represented as [quaternions](http://wiki.ros.org/tf2/Tutorials/Quaternions) which we will ignore for now,
- Positions are represented as x,y,z coordinates where the height z will remain zero.

Modify the position of the 'test_location' and check in RViz if the change matches your expectation. If you want, you can manually add additional poses. The file format is very strict, e.g. the wrong amount of spaces at the start of a line can already cause errors. 

## 2. Store and retrieve poses
We prepared a ROS2 node that can store and retrieve poses. Start it with
```bash
ros2 run mirte_location_markers pose_manager.py
```  

This will provide two new services with you can test with

```bash
ros2 service call /store_current_pose mirte_location_markers/srv/StorePose "{label: 'your_location_label'}"   
ros2 service call /get_stored_pose mirte_location_markers/srv/GetStoredPose "{pose_name: 'test_location'}"  
```

Test these services and check how they interact with the file `stored_poses.yaml`. All changes in the file should directly be reflected in RViz as well, as long as `marker_publisher.py` is running.

## 3. Test the `move_to_server`
We prepared a ROS2 node that can tell the robot to go to one of the stored locations. Collaborate with the 'navigation' team to prevent running the following command twice:
```bash
ros2 run mirte_navigation move_to_server.py
```

and test it with
```bash
ros2 service call /move_to mirte_location_markers/srv/MoveTo "{location: 'test_location'}"
```

## 4. Connect the dots manually
- Drive to a location of interest, 
- use the command-line commands shown above to store that location,
- edit the location name (e.g., call it 'desk_john')
- amaze your team members by letting Mirte Master drive autonomously from 'start' to 'desk_john' and back through two `rosservice` calls

## 5. Connect the dots programmatically
You may or may not want to use the `move_to` service. For a more complete task, e.g. move to a place, do something, then move somewhere else, maybe you can use the example file `simple_navigation_script.py`. Open it and check what it does, ask ChatGPT when in doubt.

> [!IMPORTANT]  
> Make sure that the robot cannot drive off of a table!

Execute the example script with:

```bash
ros2 run mirte_location_markers simple_navigation_script.py
```
