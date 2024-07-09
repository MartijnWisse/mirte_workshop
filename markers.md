# Markers

In this module, we will give names ('labels') to specific locations on the map. 

## Poses
In a terminal on the robot, run the following command:

`$ rosrun mirte_workshop marker_publisher_node.py`  

Collaborate with the 'RViz' team member to visualize the markers in RViz. You should see two green dots on the map, one called 'start' and the other called 'test_location'

The locations of these dots are defined in the file `~/mirte_ws/src/mirte_workshop/maps/stored_poses.yaml`. Open the file and analyse its content. 

- Orientations are represented as [quaternions](http://wiki.ros.org/tf2/Tutorials/Quaternions) which we will ignore for now,
- Positions are represented as x,y,z coordinates where the height z will remain zero.

Modify the position of the 'test_location' and check in RViz if the change matches your expectation. If you want, you can manually add additional poses. The file format is very strict, e.g. the wrong amount of spaces at the start of a line can already cause errors. 

## Store and retrieve poses
We prepared a ROS node that can store and retrieve poses. Start it with

`$ rosrun mirte_workshop pose_manager.py`  

This will provide two new services with you can test with"

```
rosservice call /store_current_pose "{}"   
rosservice call /get_stored_pose "pose_name: 'start'"   
```

Test these services and check how they interact with the file `stored_poses.yaml`. All changes in the file should directly be reflected in RViz as well, as long as the 'marker_publisher_node.py' is running.

## Test the move_to_server
We prepared a ROS node that can tell the robot to go to one of the stored locations. Collaborate with the 'navigation' team member to prevent running the following command twice:

`$ rosrun mirte_navigation move_to_server.py`

and test it with

`$ rosservice call /move_to "location: 'start'"`   


## Connect the dots manually
- Drive to a location of interest, 
- use the command-line commands shown above to store that location,
- edit the location name (e.g., call it 'desk_john')
- amaze your team members by letting Mirte Master drive autonomously from 'start' to 'desk_john' and back through two rosservice calls

## Connect the dots programmatically
It would be best to do this task together with the 'keyboard control' team member and directly implement things in their python file. However, to be able to work individually, the following script could be put into a new python file. Analyze the script to understand how it executes the same move_to motions, but now from a python script rather than from the command line. Let's call the following script `simple_navigation_script.py`.

```
#!/usr/bin/env python3

# ---- Load libraries ----
# load the library with ROS functionality for Python
import rospy      
# load the specific service format required for move_to commands
from mirte_navigation.srv import MoveTo, MoveToRequest, MoveToResponse  # Import the custom service
# load the message formats for checking the status of move_base
from actionlib_msgs.msg import GoalStatusArray, GoalStatus



class SimpleNavigationScript:
    def __init__(self):
        self.goal_finished = False
        # create the subscriber that checks whether move_base has finished reaching its goal
        self.status_subscriber = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        # create the service proxy, tell it which rosservice to call
        self.move_to = rospy.ServiceProxy('/move_to', MoveTo)
        # give the service_proxy and status subscriber some time to get ready
        rospy.sleep(0.5)  


    def status_callback(self, msg):
        # Check the status of the current goal
        for status in msg.status_list:
            if status.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST]:
                self.goal_finished = True
                rospy.loginfo("Move_base has finished its current action with status: %d", status.status)
                break
            else:
                self.goal_finished = False

    def wait_for_move_base(self):
        rospy.loginfo("Waiting for move_base to finish its current action...")
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.goal_finished:
                rospy.loginfo("Move_base has finished. Continuing with the next /move_to request...")
                break
            rate.sleep()

if __name__ == '__main__':
    # start a new ROS node
    rospy.init_node('simple_navigation_script')  
    rospy.loginfo('The simple_navigation_script has started!')  
    simple_navigation_object = SimpleNavigationScript()

    rospy.loginfo('Giving command to move to location "start"')  
    simple_navigation_object.move_to("start")
    # sleep briefly to make sure that move_base has an active goal
    rospy.sleep(0.5)
    # check for move_base to finish its first goal
    simple_navigation_object.wait_for_move_base()


    # Proceed with the rest of the operations
    rospy.loginfo('Giving command to move to location "test_location"')  
    simple_navigation_object.move_to("test_location")

```

To be able to rosrun this python file, we need to tell Linux that it is an executable file. Change directory into the folder of the file, and type

```
chmod +x simple_navigation_script.py
```

Make sure that the robot cannot drive off of a table, and execute your script with:

```
rosrun mirte_workshop simple_navigation_script.py
```
