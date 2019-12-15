#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from math import atan2, exp, sqrt, log
from math import pi as PI
import copy

class AutoConnect:
    
    def __init__(self):
        self.THETA_TOL = 0.0175  # in radian ~= 1 deg
        self.POSE_TOL = 0.01  # in meter 
        self.PUB_RATE = 30
        self.MIN_OMEGA = 0.2
        self.MIN_VEL = 0.5
        
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
    
    def faceSameDir(self, goal):
        ''' Decide to drive forward or backward '''
        if abs(self.angularDiff(goal)) < PI/2 or abs(self.angularDiff(goal)) > PI*3/2 : return True
        else : return False

    def angularDiff(self, goal):
        x_diff = goal.x - self.pose_now.x
        y_diff = goal.y - self.pose_now.y
        theta_goal = atan2(y_diff, x_diff)
        return  (theta_goal - self.theta)
        
    def angularVel(self, point, CONST=None, backward=True):
        if CONST is None: CONST = self.MIN_OMEGA
        goal = copy.deepcopy(point)
        if not self.faceSameDir(goal) and backward:
            goal.x = self.pose_now.x - (goal.x - self.pose_now.x)
            goal.y = self.pose_now.y - (goal.y - self.pose_now.y)

        theta_diff = self.angularDiff(goal)
        if theta_diff > 0:
            if theta_diff > PI: 
                return - CONST * exp(2*PI - theta_diff) 
            else : 
                return CONST * exp(theta_diff)
        if theta_diff < 0:
            if abs(theta_diff) > PI: 
                return CONST * exp(- theta_diff - 2*PI) 
            else : 
                return - CONST * exp(- theta_diff)

    def checkOrientation(self, point, backward=True):
        goal = copy.deepcopy(point)
        if not self.faceSameDir(goal) and backward :
            goal.x = self.pose_now.x - (goal.x - self.pose_now.x)
            goal.y = self.pose_now.y - (goal.y - self.pose_now.y)

        if abs(self.angularDiff(goal)) <= self.THETA_TOL: return True
        else: 
            #print("angleDiff:{0}".format(self.angularDiff(goal)))
            return False


    def euclideanDist(self, goal):
        return sqrt((goal.x - self.pose_now.x)**2 + (goal.y - self.pose_now.y)**2)

    def linearVel(self, goal, CONST=None):
        if CONST is None: CONST = self.MIN_VEL
        dist = self.euclideanDist(goal)
        if self.faceSameDir(goal) : return CONST * log(dist+1)
        elif not self.faceSameDir(goal) : return - CONST * log(dist+1)

    def move2Goal(self, goal_x, goal_y, goal_theta):

        r = rospy.Rate(self.PUB_RATE)

        cmd_vel = Twist()
        goal = Point()
        theta = 0.0

        # Target Point
        goal.x = goal_x
        goal.y = goal_y
        theta = goal_theta

        sub_goal = self.subGoal(goal)
        rospy.loginfo("Moving to : {0}".format(sub_goal))

        while not rospy.is_shutdown():
            # 1. Go to sub goal to align with the connector on B cart. 
            while not self.checkOrientation(sub_goal) or self.euclideanDist(sub_goal) > self.POSE_TOL :

                cmd_vel.linear.x = self.linearVel(sub_goal)
                cmd_vel.angular.z = self.angularVel(sub_goal)      

                if self.checkOrientation(sub_goal) :
                    cmd_vel.angular.z = 0.0
                if self.euclideanDist(sub_goal) <= self.POSE_TOL:
                    cmd_vel.linear.x = 0.0

                self.twist_pub.publish(cmd_vel)

                #print("current theta: {0}, goalThetaDiff: {1}, sameDir: {2}, {3}, {4}".format(self.theta, self.angularDiff(sub_goal),self.faceSameDir(sub_goal), self.checkOrientation(sub_goal), self.euclideanDist(sub_goal) > self.POSE_TOL))

                r.sleep()
        
    
            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            rospy.loginfo("Goal Reached!")
            rospy.loginfo("Turning for connection...")
            
            # 2. Turn to face +x toward B cart. 
            while not self.checkOrientation(goal, backward=False) :

                cmd_vel.angular.z = self.angularVel(goal, backward=False)   
                self.twist_pub.publish(cmd_vel)

                r.sleep()

            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            #print("current theta: {0}, goal theta diff: {1}, {2}".format(self.theta, self.angularDiff(sub_goal),abs(self.angularDiff(sub_goal)) > self.THETA_TOL ))

            rospy.loginfo("Connecting...")

            # 3. Connecting 
            while self.euclideanDist(goal) > self.POSE_TOL :

                cmd_vel.linear.x = self.linearVel(goal, self.MIN_VEL/2.5)
                cmd_vel.angular.z = self.angularVel(goal, self.MIN_OMEGA/10, False)      
                self.twist_pub.publish(cmd_vel)

                r.sleep()

            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            rospy.loginfo("Connected !")
            rospy.signal_shutdown("Connected!")


if __name__ == '__main__':
    
    try:
        solamr0 = AutoConnect()
        solamr0.move2Goal(2.04,1.0,0)

    except rospy.ROSInterruptException:
        pass
