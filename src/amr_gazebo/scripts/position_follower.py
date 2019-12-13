#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from math import atan2
import copy

class AutoConnect:
    
    def __init__(self):
        self.PUBRATE = 30
        self.OMEGA = 0.2
        
        self.pose_now = Point()
        self.theta = 0.0

        rospy.init_node("position_follower", anonymous=True)
        self.robot_ns = rospy.get_param('/solamr_1_follower/robot_ns') 

        self.odom_sub = rospy.Subscriber("/{0}/odom".format(self.robot_ns), Odometry, self.odomUpdate)
        self.twist_pub = rospy.Publisher("/{0}/cmd_vel".format(self.robot_ns), Twist, queue_size = 5)

    def odomUpdate(self, msg):
        ''' Update current pose ... '''
        self.pose_now.x = msg.pose.pose.position.x
        self.pose_now.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    ''' Mod required : Need to idintify where to go in for connection'''
    def subGoal(self, goal_point):
        ''' The rest point before going straight to connect ... '''
        point = copy.deepcopy(goal_point)
        if point.x > self.pose_now.x : point.x -= 1
        elif point.x < self.pose_now.x : point.x += 1
        return point


    def angularVel(self, goal):
        x_diff = goal.x - self.pose_now.x
        y_diff = goal.y - self.pose_now.y
        theta_goal = atan2(y_diff, x_diff)
        theta_diff = theta_goal - self.theta
        return self.OMEGA * theta_diff


    def move2Goal(self, goal_x, goal_y, goal_theta):

        r = rospy.Rate(self.PUBRATE)

        cmd_vel = Twist()
        goal = Point()
        theta = 0.0

        # Target Point
        goal.x = goal_x
        goal.y = goal_y
        theta = goal_theta

        sub_goal = self.subGoal(goal)
        print("Moving to :\n {0}".format(sub_goal))

        while not rospy.is_shutdown():

            cmd_vel.angular.z = self.angularVel(sub_goal)      

            self.twist_pub.publish(cmd_vel)
            r.sleep()




if __name__ == '__main__':
    
    try:
        solamr0 = AutoConnect()
        solamr0.move2Goal(1,1,0)

    except rospy.ROSInterruptException:
        pass
