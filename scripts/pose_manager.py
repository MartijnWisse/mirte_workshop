#!/usr/bin/env python3

import rospy
import yaml
import os
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mirte_navigation.srv import GetStoredPose, GetStoredPoseResponse

poses_file = '/home/mirte/mirte_ws/src/mirte_workshop/maps/stored_poses.yaml'
current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def store_current_pose(req):
    global current_pose
    if current_pose is None:
        return TriggerResponse(success=False, message="No current pose available")

    # Read existing poses
    if os.path.exists(poses_file):
        with open(poses_file, 'r') as file:
            try:
                poses = yaml.safe_load(file)
                if poses is None:
                    poses = {}
            except yaml.YAMLError as exc:
                rospy.logerr(f"Error reading YAML file: {exc}")
                poses = {}
    else:
        poses = {}

    # Determine the new label
    labels = [int(label) for label in poses.keys() if label.isdigit()]
    new_label = str(max(labels) + 1 if labels else 1)

    # Add the current pose with the new label
    print(current_pose)
    poses[new_label] = {
        'position': {
            'x': current_pose.position.x,
            'y': current_pose.position.y,
            'z': current_pose.position.z
        },
        'orientation': {
            'x': current_pose.orientation.x,
            'y': current_pose.orientation.y,
            'z': current_pose.orientation.z,
            'w': current_pose.orientation.w
        }
    }

    # Write poses back to file
    with open(poses_file, 'w') as file:
        yaml.dump(poses, file)

    return TriggerResponse(success=True, message=f"Pose stored with label {new_label}")

def get_stored_pose(req):
    label = req.pose_name
    # Read existing poses
    if os.path.exists(poses_file):
        with open(poses_file, 'r') as file:
            try:
                poses = yaml.safe_load(file)
                if poses is None or label not in poses:
                    return GetStoredPoseResponse(success=False)
            except yaml.YAMLError as exc:
                rospy.logerr(f"Error reading YAML file: {exc}")
                return GetStoredPoseResponse(success=False)
    else:
        return GetStoredPoseResponse(success=False)

    pose = poses[label]
    pose_msg = Pose()
    pose_msg.position.x = pose['position']['x']
    pose_msg.position.y = pose['position']['y']
    pose_msg.position.z = pose['position']['z']
    pose_msg.orientation.x = pose['orientation']['x']
    pose_msg.orientation.y = pose['orientation']['y']
    pose_msg.orientation.z = pose['orientation']['z']
    pose_msg.orientation.w = pose['orientation']['w']

    return GetStoredPoseResponse(success=True, pose=pose_msg)





if __name__ == '__main__':
    rospy.init_node('pose_manager')

    # Subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

    # Service servers
    rospy.Service('/store_current_pose', Trigger, store_current_pose)
    rospy.Service('/get_stored_pose', GetStoredPose, get_stored_pose)

    rospy.loginfo("Pose manager services are ready")
    rospy.spin()
