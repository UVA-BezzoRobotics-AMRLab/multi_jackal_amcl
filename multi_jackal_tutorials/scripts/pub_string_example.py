import rospy
from std_msgs.msg import String

dict_init = {
    'numAgents': 2,
    'numCities': 4,
    'startPose': [[30,30],[30,30]],
    'vels': [1, 1],
    'cityCoordinates': [[80,20],[10,80],[200,100],[150,250]],
    'numGenerations': 10,
    'populationSize': 10,
    'mutationRate': 0.1,
    'image_path': "obstacleMap2.png",
    'new_run': True
}

dict_msg = str(dict_init)
print(dict_msg)

def talker():
    pub = rospy.Publisher('string_msg', String, queue_size=10)
    rospy.init_node('init_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub.publish(dict_msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass