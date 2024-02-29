#!/usr/bin/env python3
"""Control multiple jackals using APF"""

from potential_field_class import PotentialField, get_model_pose
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import yaml, json, rospkg

real_robot = False

# Define the positions
global pos, ori, paths, pos_received, obs_info
pos = [np.array([0., 0.]), np.array([0., 0.])]
obs_info = [None, None]
pos_received = False
ori = [0., 0.]
paths = []

# Callback function for the subscriber to get relative pose information (for real robot)
def pos_cb(msg, args):
    global pos, ori, pos_received
    pos[args]    = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    ori[args]    = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    pos_received = True

def path_cb(msg):
    global paths
    paths = eval(msg.data)    
    print(paths)

def scan_cb(msg, robot_index):
    global obs_info
    min_distance = min(msg.ranges)
    min_index = msg.ranges.index(min_distance)
    incontext_angle = msg.angle_min + (min_index * msg.angle_increment)
    # normalizes the angle to 270 degrees of vision of laser scan and displaces it so that center of the blind spot is at -90 degrees
    normalized_angle = (((incontext_angle - msg.angle_min) / (msg.angle_max - msg.angle_min)) * 270) - 45
    # convert to rad
    angle = normalized_angle * (math.pi/180)
    # Store distance and angle
    obs_info[robot_index] = (min_distance, angle)

sub_names = ['jackal0/amcl_pose', 'jackal1/amcl_pose']
scan_sub_names = ['jackal0/front/scan_v2', 'jackal1/front/scan_v2']
pub_names = ['jackal0/jackal_velocity_controller/cmd_vel', 'jackal1/jackal_velocity_controller/cmd_vel']




if __name__ == '__main__':

    # Initialize the node
    rospy.init_node('multi_apf_jackal')

    # Initialize the dictionary for the environment with a yaml file
    rospack = rospkg.RosPack()
    # pkg_path = rospack.get_path('multi_jackal_tutorials')
    # pkg_path = "/home/bezzo/multi_amcl_ws/multi_jackal_tutorials"
    pkg_path = "/home/bezzo/jackal_ws/src/multi_jackal_amcl/multi_jackal_tutorials"
    with open(pkg_path + "/configs/example.yaml", "r") as stream:
        try:
            dict_init = eval(json.dumps(yaml.safe_load(stream)))
        except yaml.YAMLError as exc:
            print(exc)
    dict_init['new_run'] = True
    # Create the subscribers
    subs = [rospy.Subscriber(sub_names[i], PoseWithCovarianceStamped, pos_cb, (i)) for i in range(len(sub_names))]
    scan_subs = [rospy.Subscriber(scan_sub_names[i], LaserScan, scan_cb, (i)) for i in range(len(scan_sub_names))]

    # Create the path callback
    rospy.Subscriber('best_paths', String, path_cb)

    # Create the velocity publishers
    pubs = [rospy.Publisher(pub_names[i], Twist, queue_size=1) for i in range(len(pub_names))]

    # Publisher for the astar path planner
    template_pub = rospy.Publisher('string_msg', String, queue_size=1,latch=True)

    # Create the potential fields in a loop
    fields = [PotentialField(kappa_attr=1, kappa_rep_obs=-.1, kappa_rep_veh=1.5, d0=1.0, d1=2.0) for i in range(len(sub_names))]

    # Create the rate
    rate = rospy.Rate(10)

    #Initialize velocities
    linear_velocity  = [0, 0]
    angular_velocity = [0, 0]
    path_counter     = [0, 0]
    iter = 0

    # Main loop
    while not rospy.is_shutdown():
        # Check if best paths is empty
        if pos_received:
            # Create the message
            dict_init['startPose'] = [[pos[i][0], pos[i][1]] for i in range(len(sub_names))]
            dict_msg = str(dict_init)
            # Publish the message
            template_pub.publish(dict_msg)
            dict_init['new_run'] = False


        # Check if the paths are published by astar node
        if len(paths) < 1:
            rate.sleep()
            continue
        for i in range(len(sub_names)):
            if path_counter[i] >= len(paths[i]):
                goal = np.array(dict_init['startPose'][i])
            else:
                goal = np.array(paths[i][path_counter[i]])
            
            # Get the velocities   
            max_speed = dict_init['vels'][i]
            #TODO: The empty array next to goal should have local obstacle positions in it (or the closest obstacle)
            if obs_info[i] is not None:
                obstacle_distance, obstacle_angle = obs_info[i]
                obs_pos = np.array([pos[i][0] - (math.cos(obstacle_angle)*obstacle_distance), pos[i][1] + (math.sin(obstacle_angle)*obstacle_distance)])
            else:
                obs_pos = np.array([])
            
            linear_velocity[i], angular_velocity[i] = fields[i].get_velocities(pos[i], ori[i], goal, [], [pos[j] for j in range(len(sub_names)) if j != i], max_speed)

            # Check if the goal has been reached
            if np.linalg.norm(pos[i] - goal) < 0.3:
                path_counter[i] += 1

            # Print the velocities, goal, and position
            print("--------------------")
            print("Vehicle: ", i)
            print("Linear velocity: ", linear_velocity[i])
            print("Angular velocity: ", angular_velocity[i])
            print("Goal: ", goal)
            print("Position: ", pos[i])
            print("Closest Object: ", obs_pos)
            print("--------------------")

        # Check if all goals have been reached
        if all([path_counter[i] >= len(paths[i]) for i in range(len(sub_names))]):
            if all([np.linalg.norm(pos[i] - np.array(dict_init['startPose'][i])) < 0.1 for i in range(len(sub_names))]):
                break

        # Create the message
        for i in range(len(sub_names)):
            msg = Twist()
            msg.linear.x = linear_velocity[i]
            msg.angular.z = angular_velocity[i]
            # Publish the message
            pubs[i].publish(msg)
        
        # Sleep
        rate.sleep()