#!/usr/bin/env python3
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rospy
import numpy as np
import rospkg
import yaml, json

# Initialize the node
rospy.init_node('visualize_path')

# Read in yaml file
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('multi_jackal_tutorials')

with open(pkg_path + "/configs/example.yaml", "r") as stream:
    try:
        dict_tasks = eval(json.dumps(yaml.safe_load(stream)))
    except yaml.YAMLError as exc:
        print(exc)

# Define the global variables
assignments = []
paths = {}
def path_cb(msg):
    global paths
    paths = eval(msg.data) 
    for i in range(len(paths)):
        if len(paths[i]) == 0:
            continue
        # Create nav path array for each robot
        path_array = Path()
        path_array.header.frame_id = "map"
        path_array.header.stamp = rospy.Time.now()

        for j in range(len(paths[i])):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = paths[i][j][0]
            pose.pose.position.y = paths[i][j][1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path_array.poses.append(pose)
        pub_path_array[i].publish(path_array)


        

def assigned_tasks_cb(msg):
    global assignments
    assigned_tasks = eval(msg.data)
    assignments = []
    for i in range(len(assigned_tasks)):
        if len(assigned_tasks[i]) == 0:
            assignments.append([])
            continue
        task_list = [dict_tasks['cityCoordinates'][assigned_tasks[i][j]] for j in range(len(assigned_tasks[i]))]
        assignments.append(task_list)
        # Create marker array for each robot
        marker_array = MarkerArray()
        for j in range(len(task_list)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "robot_{0}_task".format(i)
            marker.id = j
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = task_list[j][0]
            marker.pose.position.y = task_list[j][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = dict_tasks['robotColors'][i][0]
            marker.color.g = dict_tasks['robotColors'][i][1]
            marker.color.b = dict_tasks['robotColors'][i][2]
            marker_array.markers.append(marker)
        pub_path_markers[i].publish(marker_array)
    


# Create the path callback
rospy.Subscriber('best_paths', String, path_cb)
rospy.Subscriber('assigned_tasks', String, assigned_tasks_cb)

# Create the publisher
pub_path_markers = [rospy.Publisher("task_markers_robot_{}".format(i), MarkerArray, queue_size=1,latch=True) for i in range(dict_tasks['numAgents'])]
pub_path_array = [rospy.Publisher("path_markers_robot_{}".format(i), Path, queue_size=1, latch=True) for i in range(dict_tasks['numAgents'])]

# Create the rate
rate = rospy.Rate(10)

# Main loop
if __name__ == '__main__':
    rospy.spin()
    #
