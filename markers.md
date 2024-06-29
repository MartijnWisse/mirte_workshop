# Markers

In this module, we will give names ('labels') to specific locations on the map. 

## Poses
Interpret pose.position and pose.orientation. 

## Store and retrieve poses

`$ rosrun mirte_workshop pose_manager.py`  

`$ rosservice call /store_current_pose "{}"`   
`$ rosservice call /get_stored_pose "pose_name: 'start'"`   

## Edit stored poses

stored_poses.yaml

## Show poses in RVIZ
`$ rosrun mirte_workshop marker_publisher_node.py`  

## Connect the dots
Make a python script that reads a pose from the file and then tells the robot to go there with the /move_to service.