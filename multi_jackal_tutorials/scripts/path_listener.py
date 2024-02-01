import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
global path_received 
path_received = False

def nav_path_cb(msg):
    global path_received
    path_received = True
    path = np.zeros((len(msg.poses), 2))
    for idx,pose in enumerate(msg.poses):
        path[idx,0] = pose.pose.position.x
        path[idx,1] = pose.pose.position.y
        print(pose.pose.position.x, pose.pose.position.y)

def pub_position(x, y, sub):
        """
        Publishing new initial position (x, y) --> starting point
        :param x x-position of the robot
        :param y y-position of the robot
        """
        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = rospy.get_rostime()
        initpose.header.frame_id = "map"
        initpose.pose.pose.position.x = x
        initpose.pose.pose.position.y = y
        quaternion = [0,0,0,1]
        initpose.pose.pose.orientation.w = quaternion[0]
        initpose.pose.pose.orientation.x = quaternion[1]
        initpose.pose.pose.orientation.y = quaternion[2]
        initpose.pose.pose.orientation.z = quaternion[3]
        sub.publish(initpose)
        return 

def pub_target(x, y, sub):
    """
    Publishing target point (x,y)
    :param x x-position of target
    :param y y-position of target
    """
    finalpose = PoseStamped()
    finalpose.header.stamp = rospy.get_rostime()
    finalpose.header.frame_id = "map"
    finalpose.pose.position.x = x
    finalpose.pose.position.y = y
    quaternion = [0,0,0,1]
    finalpose.pose.orientation.w = quaternion[0]
    finalpose.pose.orientation.x = quaternion[1]
    finalpose.pose.orientation.y = quaternion[2]
    finalpose.pose.orientation.z = quaternion[3]
    sub.publish(finalpose)
    return

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/nav_path", Path, nav_path_cb)
    start_point_pub = rospy.Publisher("start_point",PoseWithCovarianceStamped,queue_size=10)
    target_point_pub = rospy.Publisher("final_point",PoseStamped,queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if not path_received:
            pub_position(2,3,start_point_pub)
            pub_target(2,4,target_point_pub)
        rate.sleep()

