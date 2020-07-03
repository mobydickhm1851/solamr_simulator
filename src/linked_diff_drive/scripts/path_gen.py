#! /usr/bin/env python 

# Generate random path for diff drive
import numpy as np
import copy
import sys, select, termios, tty

import rospy
import tf.transformations as t
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
#from std_msgs.msg import Header

base = 0.2
moveBindings = {
        'i':(base,0),
        'o':(base,-2*base),
        'j':(0,2*base),
        'l':(0,-2*base),
        'u':(base,2*base),
        ',':(-base,0),
        '.':(-base,2*base),
        'm':(-base,-2*base),
           }

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class PathGen:

    def __init__(self, rot_vel_max=2.5718, # In rad/s
                       lin_vel_max=3.0, # In m/s
                       rot_acc=1.0, lin_acc=1.0): # in rad/s^2 and m/s^2
        self.path = Path()
        self.cur_pose = PoseStamped()
        self.cur_twist = Twist()
        self.PUB_INIT = False
        self.INIT_X = 0.0
        self.INIT_Y = 0.0
        self.cur_rot_vel = 0.0
        self.cur_lin_vel = 0.0
        self.cur_th = 0.0
        self.ROT_VEL = [-rot_vel_max, rot_vel_max]
        self.LIN_VEL = [-lin_vel_max, lin_vel_max]
        self.ROT_ACC = [-rot_acc, rot_acc]
        self.LIN_ACC = [-lin_acc, lin_acc]
        self.RATE = 10
        rospy.init_node('path_node')
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.f_pose_pub = rospy.Publisher('/front_pose', PoseStamped, queue_size=10)
        self.f_twist_pub = rospy.Publisher('/front_twist', Twist, queue_size=10)

    def poseGenerator(self, key='', auto=True):
        lin_a, rot_a = 0.0, 0.0
        self.cur_pose.header.stamp = rospy.Time.now()
        self.cur_pose.header.frame_id = "map"
        if not self.PUB_INIT:
            self.cur_pose.pose.position.x = self.INIT_X
            self.cur_pose.pose.position.y = self.INIT_Y
            self.PUB_INIT = True
        else :
            dt = 1.0/self.RATE
            
            ''' Randomly generate an acceleration value '''
            if auto == True:
                lin_a = np.random.uniform(self.LIN_ACC[0], self.LIN_ACC[1])
                rot_a = np.random.uniform(self.ROT_ACC[0], self.ROT_ACC[1])
                ''' control vel and omega by hand '''
            else:
                if key in moveBindings.keys():
                    lin_a = moveBindings[key][0]
                    rot_a = moveBindings[key][1]

                elif key == ' ' or key == 'k' :
                    lin_sign = abs(self.cur_lin_vel)/self.cur_lin_vel
                    rot_sign = abs(self.cur_rot_vel)/self.cur_rot_vel
                    lin_a = - lin_sign * abs(self.cur_lin_vel)
                    rot_a = - rot_sign * abs(self.cur_rot_vel)
                else:
                    if abs(self.cur_lin_vel) > 0:
                        sign = abs(self.cur_lin_vel)/self.cur_lin_vel
                        if abs(self.cur_lin_vel) < base:
                            lin_a = abs(self.cur_lin_vel)*-sign
                        else: lin_a = 1.2 * base * -sign

                    if abs(self.cur_rot_vel) > 0:
                        sign = abs(self.cur_rot_vel)/self.cur_rot_vel
                        if abs(self.cur_rot_vel) < base:
                            rot_a = abs(self.cur_rot_vel)*-sign
                        else: rot_a = 1.5 * base * -sign

                    if (key == '\x03'):
                        return "break"
                    
            ''' Check for the limits of velocity and acceleration '''
            if abs(self.cur_lin_vel + lin_a) > self.LIN_VEL[1] : 
                lin_a = 0.0 
                print("linear acceleration is 000000000000000000")
            if abs(self.cur_rot_vel + rot_a) > self.ROT_VEL[1] : 
                rot_a = 0.0 
                print("angular acceleration is 000000000000000000")

#            print(lin_a)

            x = self.cur_pose.pose.position.x
            y = self.cur_pose.pose.position.y
            vx = self.cur_lin_vel * np.cos(self.cur_th) 
            vy = self.cur_lin_vel * np.sin(self.cur_th) 
            ax = lin_a * np.cos(self.cur_th) 
            ay = lin_a * np.sin(self.cur_th) 
#            print(ax, ay)

            ''' kinematics matrix '''
            k_mat_1 = np.array([[x, vx, ax], 
                                [y, vy, ay],
                                [self.cur_th, self.cur_rot_vel, rot_a]])
            k_mat_2 = np.array([[1.0, 0.0],
                                [dt, 1.0],
                                [0.5 * dt**2, dt]])
            '''
            result matrix:
            [[x', Vx']
             [y', Vy']
             [th', omega']]
            '''
            result_mat = np.dot(k_mat_1, k_mat_2)

            ''' update the states '''
            self.cur_pose.pose.position.x = result_mat[0][0]
            self.cur_pose.pose.position.y = result_mat[1][0]
            self.cur_th = result_mat[2][0]
            self.cur_lin_vel = np.sqrt(result_mat[0][1]**2 + result_mat[1][1]**2)
            '''  
            CAN'T MOVE BACKWARD
            '''
#            if result_mat[0][1] == 0:
#                heading = 0
#            else:
#                heading = np.arctan(result_mat[1][1]/result_mat[0][1])
#            print(abs(heading - result_mat[2][0])%np.pi)
#            if abs(heading - result_mat[2][0])%np.pi > np.pi/2:
#                self.cur_lin_vel = -self.cur_lin_vel
#            print(result_mat[0][1], result_mat[1][1])
            self.cur_rot_vel = result_mat[2][1]

            ''' Turn from Euler to Quaternion '''
            result_q = t.quaternion_from_euler(0.0, 0.0, self.cur_th)
            self.cur_pose.pose.orientation.x = result_q[0]
            self.cur_pose.pose.orientation.y = result_q[1]
            self.cur_pose.pose.orientation.z = result_q[2]
            self.cur_pose.pose.orientation.w = result_q[3]

            ''' twist '''
            self.cur_twist.linear.x = self.cur_lin_vel
            self.cur_twist.angular.z = self.cur_rot_vel

#        print("speed : {0}, omega: {1}".format(self.cur_lin_vel, self.cur_rot_vel))
        return self.cur_pose

    def pathPub(self, auto=True):
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():
            ''' deepcopy is used since python handle objects by references
            if instead of deepcopy, '='(assign) is used
            all the point on the path would be the same point (last generated point) '''
            if auto==True:
                new_pose = copy.deepcopy(self.poseGenerator(auto=auto))
            else:
                k = getKey()
                new_pose = copy.deepcopy(self.poseGenerator(key=k, auto=auto))

            self.path.header = new_pose.header 
            self.path.poses.append(new_pose)
            self.path_pub.publish(self.path)

            ''' publish pose and twist of the front car '''
            self.f_pose_pub.publish(self.cur_pose)
            self.f_twist_pub.publish(self.cur_twist)

            rate.sleep()

if __name__ == '__main__':

    try:
        print("Random path generator begin!")
        new_pub_path = PathGen()
        new_pub_path.pathPub(auto=False)

    except rospy.ROSInterruptException:
        pass
