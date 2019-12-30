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

class Listener:
    
    def __init__(self, topic_name):

        rospy.init_node("Listening", anonymous=True)
        self.name = topic_name
        self.odom_sub = rospy.Subscriber("{0}".format(topic_name), Image, self.arraySizePrint)

    def arraySizePrint(self, topic):
        print("{0} has height {1} and width {2}.".format(self.name, topic.height, topic.width))


if __name__ == '__main__':
    
    while not rospy.is_shutdown(): 
        try:
            #L1 = Listener("/zed/zed_node/rgb/image_rect_color")
            #L2 = Listener("/zed/zed_node/depth/depth_registered")
            L1 = Listener("/solamr_1/camera/rgb/image_raw")
            L2 = Listener("/solamr_1/camera/depth/image_raw")

        except rospy.ROSInterruptException:
            pass
