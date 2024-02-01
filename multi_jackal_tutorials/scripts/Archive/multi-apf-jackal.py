"""Control multiple jackals using APF"""

from potential_field_class import PotentialField, get_model_pose
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


real_robot = False

# Define the positions
global pos, ori
pos = [np.array([0., 10.]), np.array([0., 10.])]
ori = [0., 0.]

# Define the goal positions (if real robot, these are relative to odometry)
goal = [np.array([10., 10.]), np.array([10., 10.])]

# Callback function for the subscriber to get relative pose information (for real robot)
def pos_cb(msg, args):
    global pos, ori, init_pos
    pos[args] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    ori[args] = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    
if real_robot:
    #TODO_CAPSTONE: change the topic names to be relative to the turtlebot
    sub_names = ['jackal0/odometry/local_filtered', 'jackal1/odometry/local_filtered']
    pub_names = ['jackal0/jackal_velocity_controller/cmd_vel', 'jackal1/jackal_velocity_controller/cmd_vel']
    # Create the subscribers
    subs = [rospy.Subscriber(sub_names[i], Odometry, pos_cb, (i)) for i in range(len(sub_names))]
else:
    sub_names = ['jackal0', 'jackal1']
    pub_names = ['jackal0/jackal_velocity_controller/cmd_vel', 'jackal1/jackal_velocity_controller/cmd_vel']

# Initialize the node
rospy.init_node('multi_apf_jackal')

# Create the publishers
pubs = [rospy.Publisher(pub_names[i], Twist, queue_size=1) for i in range(len(pub_names))]

# Create the potential fields in a loop
fields = [PotentialField(kappa_attr=1, kappa_rep_obs=10, kappa_rep_veh=10, d0=2.0, d1=3.0) for i in range(len(sub_names))]

# Create the rate
rate = rospy.Rate(10)

#Initialize velocities
linear_velocity = [0, 0]
angular_velocity = [0, 0]

# Main loop
while not rospy.is_shutdown():
    # Get the velocities
    for i in range(len(sub_names)):
        if not real_robot:
            pos_hold = get_model_pose(sub_names[i])
            pos[i] = np.array([pos_hold.position.x, pos_hold.position.y])
            # Change orientation from quaternion to euler
            ori[i] = euler_from_quaternion([pos_hold.orientation.x, pos_hold.orientation.y, pos_hold.orientation.z, pos_hold.orientation.w])[2]
        if not real_robot:
            linear_velocity[i], angular_velocity[i] = fields[i].get_velocities(pos[i], ori[i], goal[i], [], [pos[j] for j in range(len(sub_names)) if j != i], 1.0)
        else: 
            #TODO: transform other vehicle positions to be relative to the current vehicle
            linear_velocity[i], angular_velocity[i] = fields[i].get_velocities(pos[i], ori[i], goal[i], [], [], 1.0)
        print("-------------------")
        print("Robot", i)
        print("Linear Velocity:", linear_velocity[i])
        print("Angular Velocity:", angular_velocity[i])
        print("Position:", pos[i])
        print("-------------------")


    # Create the message
    for i in range(len(sub_names)):
        msg = Twist()
        # msg.data = [linear_velocity[i], angular_velocity[i]]
        msg.linear.x = linear_velocity[i]
        msg.angular.z = angular_velocity[i]
        # Publish the message
        pubs[i].publish(msg)
    
    # Sleep
    rate.sleep()