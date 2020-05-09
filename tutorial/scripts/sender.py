#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sender():
    pub = rospy.Publisher('chat', String, queue_size=10)
    rospy.init_node('sender', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = "hello world %s" % rospy.get_time()
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
