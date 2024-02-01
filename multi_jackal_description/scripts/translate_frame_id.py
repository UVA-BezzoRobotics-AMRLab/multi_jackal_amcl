#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(msg, args):
    msg.header.frame_id = 'jackal' + str(args) + '/front_laser'
    pub = rospy.Publisher('/jackal'+ str(args) +'/front/scan_v2', LaserScan, queue_size=1,latch=True)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('transform_frame', anonymous=True)
    sub_names = ['jackal0','jackal1']
    subs = [rospy.Subscriber('/'+ sub_names[i] +'/front/scan', LaserScan, callback, i) for i in range(len(sub_names))]
    rate = rospy.Rate(100)
    rospy.spin()