#!/usr/bin/env python3

import rospy
import rospkg
import yaml
import os
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

class StoredPointsVisualizer:
    def __init__(self):
        rospy.init_node('stored_points_visualizer')
        self.marker_pub = rospy.Publisher('/stored_points_markers', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('mirte_workshop')
        self.locations_dir = os.path.join(self.package_path, "maps")
        self.previous_markers = set()
        rospy.loginfo("Topic '/stored_points_markers' now available to show as MarkerArray in RViz. Edit the stored points in ./maps/stored_poses.yaml")

    def load_locations(self, map_file_name):
        locations_file_path = os.path.join(self.locations_dir, f'{map_file_name}.yaml')
        if not os.path.exists(locations_file_path):
            #rospy.logwarn(f"No locations file found for {map_file_name}")
            return []
        
        with open(locations_file_path, 'r') as file:
            locations_data = yaml.safe_load(file)
        
        if not locations_data:
            #rospy.logwarn(f"No locations data in file {locations_file_path}")
            return []

        return locations_data

    def create_marker(self, marker_type, id, position, orientation, scale, color, frame_id="map", action=Marker.ADD):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "stored_points"
        marker.id = id
        marker.type = marker_type
        marker.action = action
        marker.pose.position = position
        marker.pose.orientation = orientation
        marker.scale = scale
        marker.color = color
        return marker

    def publish_markers(self):
        while not rospy.is_shutdown():
            map_file_name = 'stored_poses'
            locations_data = self.load_locations(map_file_name)

            marker_array = MarkerArray()
            current_marker_ids = set()

            for i, (key, location) in enumerate(locations_data.items()):
                position = Point(location['position']['x'], location['position']['y'], location['position']['z'])
                orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
                vector_orientation = Quaternion(
                    location['orientation']['x'],
                    location['orientation']['y'],
                    location['orientation']['z'],
                    location['orientation']['w']
                )
                scale_cylinder = Vector3(0.1, 0.1, 0.02)  # 10 cm radius, 2 cm height
                scale_arrow = Vector3(0.2, 0.01, 0.01)  # 20 cm length, 1 cm shaft radius
                color_green = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green color

                cylinder_marker = self.create_marker(Marker.CYLINDER, i, position, orientation, scale_cylinder, color_green)
                arrow_marker = self.create_marker(Marker.ARROW, i + len(locations_data), position, vector_orientation, scale_arrow, color_green)

                text_position = Point(position.x, position.y, position.z + 0.2)  # Slightly above the point
                scale_text = Vector3(0.0, 0.0, 0.2)  # Text height
                color_black = ColorRGBA(0.0, 0.0, 0.0, 1.0)  # Black color
                text_marker = self.create_marker(Marker.TEXT_VIEW_FACING, i + 2 * len(locations_data), text_position, orientation, scale_text, color_black)
                text_marker.text = location.get('label', f'{key}')

                current_marker_ids.update([i, i + len(locations_data), i + 2 * len(locations_data)])

                marker_array.markers.append(cylinder_marker)
                marker_array.markers.append(arrow_marker)
                marker_array.markers.append(text_marker)

            # Find markers to delete
            markers_to_delete = self.previous_markers - current_marker_ids
            for marker_id in markers_to_delete:
                delete_marker = Marker()
                delete_marker.header.frame_id = "map"
                delete_marker.header.stamp = rospy.Time.now()
                delete_marker.ns = "stored_points"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

            self.marker_pub.publish(marker_array)
            self.previous_markers = current_marker_ids

            self.rate.sleep()


if __name__ == '__main__':
    visualizer = StoredPointsVisualizer()
    visualizer.publish_markers()
