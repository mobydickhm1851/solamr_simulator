#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from math import atan2, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class Publisher:
    
    def __init__(self, topic_name):

        rospy.init_node("Listening", anonymous=True)
        self.test_pub = rospy.Publisher("{0}".format(topic_name), Float64MultiArray, queue_size=5)

    def array_publisher(self):
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label='id number'
        msg.layout.dim[0].size=2*2
        msg.data = [1.0, 2.0]
        self.test_pub.publish(msg)


if __name__ == '__main__':
    
    while not rospy.is_shutdown(): 
        try:
            test = Publisher("error_lst")
            test.array_publisher()

        except rospy.ROSInterruptException:
            pass
