#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf2_ros
from math import atan2, asin, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np
#from tf.transformations import euler_from_quaternion
#from geometry_msgs.msg import Point, TransformStamped
#from nav_msgs.msg import Odometry

class Imu2Theta:
    
    def __init__(self, num):

        self.imu_sub = rospy.Subscriber("/2in1/imu_{0}".format(num), Imu, self.imu_cb)
        self.theta_pub = rospy.Publisher("/imu_{0}/theta".format(num), Float64, queue_size=10)

    def imu_cb(self, imu_data):
        # Read the quaternion of the robot IMU
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w

        # Read the angular velocity of the robot IMU
        w_x = imu_data.angular_velocity.x
        w_y = imu_data.angular_velocity.y
        w_z = imu_data.angular_velocity.z

        # Read the linear acceleration of the robot IMU
        a_x = imu_data.linear_acceleration.x
        a_y = imu_data.linear_acceleration.y
        a_z = imu_data.linear_acceleration.z

        # Convert Quaternions to Euler-Angles
        rpy_angle = [0, 0, 0]
        rpy_angle[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        rpy_angle[1] = asin(2 * (w * y - z * x))
        rpy_angle[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        self.theta_pub.publish(rpy_angle[2])

if __name__ == '__main__':
    
    while not rospy.is_shutdown(): 
        try:
            rospy.init_node("imu2theta", anonymous=True)
            imu1 = Imu2Theta(1)
            imu2 = Imu2Theta(2)
            rospy.spin()


        except rospy.ROSInterruptException:
            pass
