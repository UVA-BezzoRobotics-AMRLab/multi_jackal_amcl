#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from potential_field_class import get_model_pose

def publish_world_odom(pose, pub):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = pose.position.x
        msg.pose.position.y = pose.position.y
        msg.pose.position.z = pose.position.z
        msg.pose.orientation.x = pose.orientation.x
        msg.pose.orientation.y = pose.orientation.y
        msg.pose.orientation.z = pose.orientation.z
        msg.pose.orientation.w = pose.orientation.w
        pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('world_odom_publisher')
    pub_names = ['jackal0/world_pose', 'jackal1/world_pose']
    sub_names = ['jackal0','jackal1']
    pubs = [rospy.Publisher(pub_names[i], PoseStamped, queue_size=1) for i in range(len(pub_names))]
    rate = rospy.Rate(30)
    while not rospy.is_shutdown(): 
        pose = [get_model_pose(sub_names[i]) for i in range(len(sub_names))]      
        for pub in pub_names:
            publish_world_odom(pose, pub)
        rate.sleep()