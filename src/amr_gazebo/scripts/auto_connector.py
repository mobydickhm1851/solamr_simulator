#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
import tf.transformations as t
from geometry_msgs.msg import Point
from math import atan2, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import sys

class AutoConnect:
    
    def __init__(self):
        self.THETA_TOL = 0.0175  # in radian ~= 2 deg
        self.POSE_TOL = 0.01  # in meter 
        self.PUB_RATE = 30
        self.MAX_OMEGA = PI/3
        self.MAX_VEL = .5
        self.MIN_OMEGA = 0.001
        self.MIN_VEL = 0.001
        self.MIN_OMEGA_RATIO = 0.2
        self.MIN_VEL_RATIO = 0.5
        self.pose_now = Point()
        self.theta = 0.0
        self.shelft_dict = dict()

        rospy.init_node("position_follower", anonymous=True)
        #self.robot_ns = rospy.get_param('/solamr_1_AC/robot_ns') 
        # FAKE ns
        self.robot_ns = "solamr_1"
        '''tf listener and '''
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.fiducial_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.aruco2Pose)
        self.odom_sub = rospy.Subscriber("/{0}/odom".format(self.robot_ns), Odometry, self.odomUpdate)
        self.twist_pub = rospy.Publisher("/{0}/cmd_vel".format(self.robot_ns), Twist, queue_size = 5)
        self.shelft_pose_pub = rospy.Publisher("/shelft_pose", Float64MultiArray, queue_size = 5)


    def vectorRotateQuaternion(self, q, v):
        '''return  qvq^-1.  q(x, y, z, w) '''
        #t = tf.transformations 
        qua_mul = t.quaternion_multiply
        q_c = t.quaternion_conjugate(q)
        return qua_mul( qua_mul(q, v), q_c )

    def odomUpdate(self, msg):
        ''' Update current car pose ... '''
        self.pose_now.x = msg.pose.pose.position.x
        self.pose_now.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = t.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    ''' Mod required : Need to idintify where to go in for connection'''
    def subGoal(self, goal_point, goal_theta, dist=1.0):
        ''' The rest point before going straight to connect ... '''
        theta = goal_theta  
        point = copy.deepcopy(goal_point)
        dx = dist * np.cos(theta)
        dy = dist * np.sin(theta)
        point.x += dx
        point.y += dy
        return point
    
    def faceSameDir(self, goal):
        ''' Decide to drive forward or backward '''
        if abs(self.pointAngularDiff(goal)) < PI/2 or abs(self.pointAngularDiff(goal)) > PI*3/2 : 
            return True # same dir, drive forward
        else : return False # opposite dir, drive reverse

    def checkSudoGoal(self, point, backward):
        ''' check if oppisite direction sudo-goal is needed'''
        goal = copy.deepcopy(point)
        if not self.faceSameDir(goal) and backward:
            goal.x = self.pose_now.x - (goal.x - self.pose_now.x)
            goal.y = self.pose_now.y - (goal.y - self.pose_now.y)
        return goal

    def pointAngularDiff(self, goal):
        x_diff = goal.x - self.pose_now.x
        y_diff = goal.y - self.pose_now.y
        theta_goal = atan2(y_diff, x_diff)
        return  (theta_goal - self.theta)

    def angularDiff(self, goal_th):
        return  (goal_th - self.theta)
        
    def angularVel(self, point, RATE=None, goal_th=None, backward=True):
        if RATE is None: RATE = self.MIN_OMEGA_RATIO
        goal = self.checkSudoGoal(point, backward)
        if goal_th is None: theta_diff = self.pointAngularDiff(point)
        else : theta_diff = self.angularDiff(goal_th)
        ''' prevent oscilliation '''
        if abs(theta_diff) < self.THETA_TOL*2 : RATE*=0.05
        elif abs(theta_diff) < self.THETA_TOL*5 : RATE*=0.2
        ''' turn CW or CCW '''
        if theta_diff > 0:
            if theta_diff > PI: 
                ang_temp =  - RATE * exp(2*PI - theta_diff) 
            else : 
                ang_temp =  RATE * exp(theta_diff)
        if theta_diff < 0:
            if abs(theta_diff) > PI: 
                ang_temp = RATE * exp(2*PI + theta_diff) 
            else : 
                ang_temp = - RATE * exp(- theta_diff)
        if abs(ang_temp) >= self.MAX_OMEGA: ang_temp = self.MAX_OMEGA * abs(ang_temp)/ang_temp
        elif abs(ang_temp) <= self.MIN_OMEGA: ang_temp = self.MIN_OMEGA * abs(ang_temp)/ang_temp
        return ang_temp

    def checkOrientation(self, point, goal_th=None, backward=True):
        ''' more tolerance is accepted if only going to that direction, not final turning '''
        goal = self.checkSudoGoal(point, backward)
        if goal_th is None: 
            theta_diff = self.pointAngularDiff(point)
            if abs(theta_diff) <= 5 * self.THETA_TOL: return True
            else: return False
        else : 
            theta_diff = self.angularDiff(goal_th)
            if abs(theta_diff) <= self.THETA_TOL: return True
            else: return False


    def euclideanDist(self, goal):
        return sqrt((goal.x - self.pose_now.x)**2 + (goal.y - self.pose_now.y)**2)

    def linearVel(self, goal, RATE=None):
        if RATE is None: RATE = self.MIN_VEL_RATIO
        dist = self.euclideanDist(goal)
        if self.faceSameDir(goal) : 
            vel_temp = RATE * log(dist+1)
        elif not self.faceSameDir(goal) : 
            vel_temp = - RATE * log(dist+1)
        # MIN and MAX
        if abs(vel_temp) >= self.MAX_VEL: vel_temp = self.MAX_VEL * abs(vel_temp)/vel_temp
        elif abs(vel_temp) <= self.MIN_VEL: vel_temp = self.MIN_VEL * abs(vel_temp)/vel_temp
        return vel_temp

    def tf2pose(self, b_pose, b_th, l_x, l_y): 
        ''' from transform to global pose:base_pose, base_theta, local x and y '''
        g_pose = Point()
        rot_mat = np.array([[np.cos(b_th), -np.sin(b_th)]
                ,[np.sin(b_th), np.cos(b_th)]])
        [g_pose.x, g_pose.y] = np.dot(rot_mat, [l_x, l_y])+[b_pose.x, b_pose.y]
        return g_pose

    def aruco2Pose(self, msg):
        ''' listen to tf transform and add shelft id and pose into dic '''

        r = rospy.Rate(self.PUB_RATE)
        ''' subscrib all aruco found and save their pose '''
        for m in msg.transforms:
            id = m.fiducial_id
            try:
                ''' get the tf transform from aruco to base '''
                tf2 = self.tfBuffer.lookup_transform(
                        "{0}/base_footprint".format(self.robot_ns), 
                        "fid{0}".format(id),
                        rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
                tf2 = None
                r.sleep()

            if tf2 != None:
                trans = tf2.transform.translation
                rot = tf2.transform.rotation #only x, y is needed
                (roll, pitch, id_theta) = t.euler_from_quaternion(
                                        [rot.x, rot.y, rot.z, rot.w])

                #print("id_theta1 of {1} : {0}".format(id_theta, id))

                ''' get the pose of the aruco '''
                id_pose = self.tf2pose(self.pose_now, self.theta, trans.x, trans.y)
                ''' get the orientation of z-axis of aruco (x, y, z, w)'''
                z_vec = self.vectorRotateQuaternion([rot.x, rot.y, rot.z, rot.w], [0.0, 0.0, 1.0, 0.0])
                ''' project z_vec to x-y plane and get the angle in atan2 '''
                id_theta = atan2(z_vec[1], z_vec[0])

                #print("id_theta2 {1} : {0}".format(id_theta, id))
                
                ''' get the center of the shelft (id to shelft center 0.45/2)'''
                id_pose.x += 0.45/2 * np.cos(id_theta + PI) 
                id_pose.y += 0.45/2 * np.sin(-id_theta + PI) 
                self.shelft_dict[id] = [id_pose, id_theta] # point, float
                #print("{1} shelft {0}".format(self.shelft_dict[id], id))
                '''car and estimated shelft pose in [car_x, car_y, est_x, est_y] '''
                msg = Float64MultiArray()
                msg.layout.dim.append(MultiArrayDimension())
                msg.layout.dim[0].label='id number:{0}'.format(id)
                msg.layout.dim[0].size= 1*5
                msg.data = [self.pose_now.x, self.pose_now.y, id_pose.x, id_pose.y, id_theta]
                self.shelft_pose_pub.publish(msg)
                r1 = rospy.Rate(1)
                r1.sleep()
            

    def move2Goal(self, goal, theta):
        ''' move to the designated pose with yaw as theta '''

        r = rospy.Rate(self.PUB_RATE)
        cmd_vel = Twist()
        xyREACHED = False
        straightDIST = 1.0 # meter

        while not self.checkOrientation(goal, theta) or self.euclideanDist(goal) > self.POSE_TOL :
            DIST2GOAL = self.euclideanDist(goal)
            cmd_vel.linear.x = self.linearVel(goal)
            print("dist to goal = {0}".format(self.euclideanDist(goal)))

            if DIST2GOAL > self.POSE_TOL : 
                cmd_vel.angular.z = self.angularVel(goal) 
                print("theta diff = {0}".format(self.pointAngularDiff(goal)))
            else : 
                cmd_vel.angular.z = self.angularVel(goal, goal_th=theta) 
                print("theta diff = {0}".format(self.angularDiff(theta)))

            if DIST2GOAL <= straightDIST and not self.checkOrientation(goal):
                cmd_vel.linear.x = 0.0

            ''' set to 0 when goal pose or orientation is reached '''
            if self.checkOrientation(goal, theta) :
                cmd_vel.angular.z = 0.0
            if DIST2GOAL <= self.POSE_TOL or xyREACHED:
                cmd_vel.linear.x = 0.0
                xyREACHED = True

            self.twist_pub.publish(cmd_vel)
            r.sleep()

        cmd_vel.linear.x = 0.0      
        cmd_vel.angular.z = 0.0      
        self.twist_pub.publish(cmd_vel)
        print("Goal Reached")
        

    

    def deprecated_move2Goal(self, id_num):

        r = rospy.Rate(self.PUB_RATE)

        # Target Point
        goal = self.shelft_dict[id_num][0] 
        # using subgoal to be parallel to B cart's connector
        theta = self.shelft_dict[id_num][1] 
        cmd_vel = Twist()


        while not rospy.is_shutdown():
            ''' 0. Moving closer (2m) for more accurate pose estimation'''
            sub_goal = self.subGoal(goal,theta,2)
            rospy.loginfo("Moving to {0} for more accurate pose".format(sub_goal))
            while not self.checkOrientation(sub_goal, theta) or self.euclideanDist(sub_goal) > self.POSE_TOL :

                cmd_vel.linear.x = self.linearVel(sub_goal, theta)
                cmd_vel.angular.z = self.angularVel(sub_goal, theta)      
                # set to 0 when goal pose or orientation is reached 
                if self.checkOrientation(sub_goal, theta) :
                    cmd_vel.angular.z = 0.0
                if self.euclideanDist(sub_goal) <= self.POSE_TOL:
                    cmd_vel.linear.x = 0.0

                self.twist_pub.publish(cmd_vel)
                r.sleep()
    
            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            rospy.loginfo("Estimating Shelft Pose!")

            # update the shelft pose before connection 
            goal = self.shelft_dict[id_num][0] 
            theta = self.shelft_dict[id_num][1] 
            rospy.loginfo("Now shelft {0} at: {1}".format(id_num, goal))
            sub_goal = self.subGoal(goal,theta,1)
            rospy.loginfo("Moving to : {0}".format(sub_goal))

            ''' 1. Go to sub goal to align with the connector on B cart.''' 
            while not self.checkOrientation(sub_goal, theta) or self.euclideanDist(sub_goal) > self.POSE_TOL :

                cmd_vel.linear.x = self.linearVel(sub_goal, theta)
                cmd_vel.angular.z = self.angularVel(sub_goal, theta)      
                # set to 3 when goal pose or orientation is reached 
                if self.checkOrientation(sub_goal, theta) :
                    cmd_vel.angular.z = 0.0
                if self.euclideanDist(sub_goal) <= self.POSE_TOL:
                    cmd_vel.linear.x = 0.0

                self.twist_pub.publish(cmd_vel)
                r.sleep()
        
    
            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            rospy.loginfo("Goal Reached!")

            rospy.loginfo("Turning for connection...")
            
            ''' 2. Turn to face +x toward B cart. '''
            while not self.checkOrientation(goal, theta, backward=False) :

                cmd_vel.angular.z = self.angularVel(goal, theta, backward=False)   
                self.twist_pub.publish(cmd_vel)

                r.sleep()

            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)

            rospy.loginfo("Connecting...")

            ''' 3. Connecting '''
            while self.euclideanDist(goal) > self.POSE_TOL :

                cmd_vel.linear.x = self.linearVel(goal, theta, self.MIN_VEL_RATIO/2.5)
                cmd_vel.angular.z = self.angularVel(goal, theta, self.MIN_OMEGA_RATIO/10, False)      
                self.twist_pub.publish(cmd_vel)

                r.sleep()

            cmd_vel.linear.x = 0.0      
            cmd_vel.angular.z = 0.0      
            self.twist_pub.publish(cmd_vel)
            rospy.loginfo("Connected !")
            rospy.signal_shutdown("Connected!")
            break


    def id2GoalPose(self, shelft_id):
        ''' get shelft pose from id '''
        pass



if __name__ == '__main__':
    from pynput import keyboard
    COMBINATIONS = [
        {keyboard.Key.ctrl, keyboard.KeyCode(char='g')},
        {keyboard.Key.ctrl, keyboard.KeyCode(char='G')}
    ]

    current = set()

    def execute():
        print("moving?")
        goal_pt = Point()
        goal_pt.x = -1
        goal_pt.y = 2
        solamr.move2Goal(goal_pt, PI)
        print("Done MOving")

        #print(solamr0.shelft_dict)
        #input_num = 202
        #goal = solamr0.shelft_dict[202][0]
        #theta = solamr0.shelft_dict[202][1]
        #print(solamr0.subGoal(goal, theta, 2))
        #solamr0.move2Goal(input_num)

    def on_press(key):
        if any([key in COMBO for COMBO in COMBINATIONS]):
            current.add(key)
            if any(all(k in current for k in COMBO) for COMBO in COMBINATIONS):
                execute()

    def on_release(key):
        if any([key in COMBO for COMBO in COMBINATIONS]):
            current.remove(key)
        elif key == keyboard.Key.esc:
            return False  # to end the listener

    try:
        solamr = AutoConnect()
        print("Press Ctrl-G to begin !")
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
        #rospy.spin()
    
    except rospy.ROSInterruptException:
        pass
