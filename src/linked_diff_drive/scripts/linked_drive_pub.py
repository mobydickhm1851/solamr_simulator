#! /usr/bin/env python 

# Generate random path for diff drive
from math import atan2, exp, sqrt, log
from math import pi as PI
import numpy as np
import copy
import time

import rospy
import tf.transformations as t
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist


class LinkedDrive:

    def __init__(self, rot_vel_max=3.5718, # In rad/s
                       lin_vel_max=5.0, # In m/s
                       rot_acc=3.0, lin_acc=3.0): # in rad/s^2 and m/s^2

        ''' Optimization Ratios '''
        self.RATIO_DIST = 1.0

        ''' Shared params '''
        self.dt = 0.3 # sec
        self.L = 1.0 # dist between two solamr when connected with shelft
        self.DIST_ERR = 0.05
        self.ANG_RES = 0.01

        ''' variables of FRONT car '''
        self.RATE = 10
        self.front_pose = [0.0, 0.0] # [x, y]
        self.front_ori = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
        self.front_th = 0.0
        self.front_vel = [0.0, 0.0]  # [v_linear, w_angular]
        self.front_predict_vel = [0.0, 0.0]  # [v_linear, w_angular]
        
        ''' variables and params of REAR car '''
        self.INIT_X = - self.L
        self.INIT_Y = 0.0
        self.cur_pose = [self.INIT_X, self.INIT_Y] # [x, y]
        self.cur_vel = [0.0, 0.0] # [lin, ang]
        self.cur_ori = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
        self.cur_th = 0.0
        self.ROT_VEL = [-rot_vel_max, rot_vel_max]
        self.LIN_VEL = [-lin_vel_max, lin_vel_max]
        self.ROT_ACC = [-rot_acc, rot_acc]
        self.LIN_ACC = [-lin_acc, lin_acc]

        ''' ROS node '''
        self.f_sub_1 = rospy.Subscriber("/solamr_1/odom", Odometry, self.f_pose_update)
        self.f_sub_2 = rospy.Subscriber("/solamr_1/cmd_vel", Twist, self.fp_vel_update)
        self.r_sub_1 = rospy.Subscriber("/solamr_2/odom", Odometry, self.r_pose_update)
        self.r_vel_pub = rospy.Publisher('/solamr_2/cmd_vel', Twist, queue_size=10)

    def fp_vel_update(self, data):
        self.front_predict_vel[0] = data.linear.x
        self.front_predict_vel[1] = data.angular.z

    def f_pose_update(self, data):
        ''' update x, y coordinate '''
        self.front_pose[0] = data.pose.pose.position.x
        self.front_pose[1] = data.pose.pose.position.y
        
        ''' update vels '''
        self.front_vel[0] = data.twist.twist.linear.x
        self.front_vel[1] = data.twist.twist.angular.z

        ''' update quaternion '''
        self.front_ori[0] = data.pose.pose.orientation.x
        self.front_ori[1] = data.pose.pose.orientation.y
        self.front_ori[2] = data.pose.pose.orientation.z
        self.front_ori[3] = data.pose.pose.orientation.w

        (row, pitch, self.front_th) = t.euler_from_quaternion(self.front_ori)

    def r_pose_update(self, data):
        ''' update x, y coordinate '''
        self.cur_pose[0] = data.pose.pose.position.x
        self.cur_pose[1] = data.pose.pose.position.y
        
        ''' update quaternion '''
        self.cur_ori[0] = data.pose.pose.orientation.x
        self.cur_ori[1] = data.pose.pose.orientation.y
        self.cur_ori[2] = data.pose.pose.orientation.z
        self.cur_ori[3] = data.pose.pose.orientation.w

        (row, pitch, self.cur_th) = t.euler_from_quaternion(self.cur_ori)

    def pose_update(self, pose, vel, th0):
        ''' prediction of front car's trajectory using arc (combination of v and w) '''
        ''' front car state 0 '''
        x0 = pose[0]
        y0 = pose[1]
        ''' radius of the arc : r > 0, arc on the left; r < 0, arc on the right'''
        omega = vel[1]
        if omega == 0:
            r = 0
            d_x = vel[0] * self.dt * np.cos(th0)
            d_y = vel[0] * self.dt * np.sin(th0)
        else:
            r = vel[0] / omega
            ''' pose of end of arc before rotation '''
            d_x = r * np.sin(omega * self.dt) 
            d_y = r - r * np.cos(omega * self.dt) 
#        print("d_x, d_y in predict={0}".format([d_x, d_y]))
        ''' rotate for the theta '''
        PI = np.pi
        rot_mat = np.array([[np.cos(th0), - np.sin(th0)],
                            [np.sin(th0), np.cos(th0)]])
        [[x1], [y1]] = [[x0], [y0]] + np.dot(rot_mat, [[d_x], [d_y]])
        th1 = th0 + omega * self.dt 
#        print("\ncurrent : {0}".format([f_x_0, f_y_0]))
#        print("\ncurrent vel : {0}".format(self.front_vel))
#        print("\npredicted : {0}".format([f_x_1, f_y_1]))
        
        return [x1, y1, th1]
        
    def get_potential_poses(self):
        ''' return potential locations (poses) for follower to be at '''
        res = self.ANG_RES # rad, potential poses every __ rad
        ''' get predicted front pose from front's current vels'''
        [x, y] = self.pose_update(self.front_pose, self.front_vel, self.front_th)[:2]
#        ''' get predicted front pose from front's predicted vels'''
#        [x, y] = self.pose_update(self.front_pose, self.front_predict_vel, self.front_th)[:2]
        lst_rad = np.arange(0, 2 * np.pi, res)
#        print("potential poses = {0}".format(len(lst_rad)))
        lst_poses = []
        for th in lst_rad:
            lst_poses.append([x + self.L * np.cos(th), y + self.L * np.sin(th)])
        return lst_poses

    def vels_from_pose(self, poses):
        ''' return lin/ang velocities from given pose '''
        dict_reachable = dict()
        mat_rot_inv = np.array([ [np.cos(self.cur_th), np.sin(self.cur_th)],
                                 [-np.sin(self.cur_th), np.cos(self.cur_th)] ])

        for pose in poses:
            [[dx], [dy]] = np.dot(mat_rot_inv, np.array([[pose[0]-self.cur_pose[0]],
                                                         [pose[1]-self.cur_pose[1]]]))
            if dy == 0: # only lin_vel, no rotation
                w = 0.0
                v = dx / self.dt
            else:
                r = (dx**2 + dy**2) / (2.0*dy)
                ''' w = omega and v = vel.x '''
                w = np.arcsin(dx /np.array(r)) / self.dt # 1x2 mat
                v = np.array(r) * np.array(w)

#            print("pose={0}; v and w={1}; check_result={2}; dxdy={3}".format(pose, [v,w], self.check_vels_range([v,w]), [dx,dy]))
            temp_pose = self.pose_update(self.cur_pose, [v,w], self.cur_th)[:2]
            if self.check_vels_range([v, w]) and self.dist2pose(temp_pose, pose) < 0.01: 
                dict_reachable[tuple(pose)] = [v, w]

        return dict_reachable

    def check_vels_range(self, vels):
        ''' check if the [v, w] is within the bounds '''
        v1, w1 = vels
        v0, w0 = self.cur_vel
        av, aw = (np.array(vels) - np.array(self.cur_vel)) / self.dt
        flag = 0
        if v1 < self.LIN_VEL[0] or v1 > self.LIN_VEL[1]:
#            print("linear velocity exceeds the range!!")
            flag += 1
        if w1 < self.ROT_VEL[0] or w1 > self.ROT_VEL[1]:
#            print("angular velocity exceeds the range!!")
            flag += 1
        if av < self.LIN_ACC[0] or av > self.LIN_ACC[1]:
#            print("linear acceleration exceeds the range!!")
            flag += 1
        if aw < self.ROT_ACC[0] or aw > self.ROT_ACC[1]:
#            print("angular acceleration exceeds the range!!")
            flag += 1

        if flag == 0: return True
        else: return False
        
    def pointAngularDiff(self, goal):
        x_diff = goal[0] - self.cur_pose[0]
        y_diff = goal[1] - self.cur_pose[1]
        theta_goal = atan2(y_diff, x_diff)
        return  (theta_goal - self.cur_th)

    def angularVel(self, point, RATE_ang=0.5, backward=True):
        THETA_TOL = 0.0175  # in radian ~= 2 deg
        MAX_OMEGA = self.ROT_VEL[1]
        MIN_OMEGA = 0.1
        theta_diff = self.pointAngularDiff(point)
        ''' TEST: see if this can follow better '''
#        theta_diff -= PI/8
        ang_temp = 0.0000000001
        ''' prevent oscilliation '''
        if abs(theta_diff) < THETA_TOL*2 : RATE_ang*=0.3
        elif abs(theta_diff) < THETA_TOL*11 : RATE_ang*=0.5
        ''' turn CW or CCW '''
        if theta_diff > 0:
            if theta_diff > PI: 
                ang_temp =  - RATE_ang * exp(2*PI - theta_diff) 
            else : 
                ang_temp =  RATE_ang * exp(theta_diff)
        if theta_diff < 0:
            if abs(theta_diff) > PI: 
                ang_temp = RATE_ang * exp(2*PI + theta_diff) 
            else : 
                ang_temp = - RATE_ang * exp(- theta_diff)
        if abs(ang_temp) >= MAX_OMEGA: ang_temp = MAX_OMEGA * abs(ang_temp)/ang_temp
#        elif abs(ang_temp) <= MIN_OMEGA: ang_temp = MIN_OMEGA * abs(ang_temp)/ang_temp
        return ang_temp
    
    def faceSameDir(self, goal):
        ''' Decide to drive forward or backward '''
        if abs(self.pointAngularDiff(goal)) < PI/2 or abs(self.pointAngularDiff(goal)) > PI*3/2 : 
            return True # same dir, drive forward
        else : return False # opposite dir, drive reverse

    def dist2pose(self, pose1, pose2):
        return np.sqrt(sum((np.array(pose1) - np.array(pose2))**2))

    def linearVel(self, goal, RATE_lin=0.5):
        dist = self.dist2pose(self.cur_pose, goal)
        if self.faceSameDir(goal) : 
            vel_temp = RATE_lin * log(dist+1)
        elif not self.faceSameDir(goal) : 
            vel_temp = - RATE_lin * log(dist+1)
        ''' MIN and MAX '''
        if abs(vel_temp) >= self.LIN_VEL[1]: vel_temp = self.LIN_VEL[1] * abs(vel_temp)/vel_temp
        return vel_temp

    def rate_dist(self, target_pose):
        ''' rate the distance between front and follower (closer the lower) '''
        r_d = self.RATIO_DIST
        follower_pose = self.cur_pose
        return r_d * self.dist2pose(follower_pose, target_pose)

    def rate_ori(self, target_pose, mode="shelft"):
        ''' rate the orientation, standatd: 1.shelft orientation, 2.front car orientation '''
        
        ''' 1.rate by shelft orientation '''

        ''' 2.rate by front car orientation '''

        pass
    
    def pose_optimization(self):
        ''' return optimized pose from reachable poses '''
        potential_poses = self.get_potential_poses()
        dict_reachable = self.vels_from_pose(potential_poses)
        reachable_poses = dict_reachable.keys()
        print(dict_reachable)

        if len(reachable_poses) > 0:
            dict_cost = dict()
            ''' optimization according to : dist to target, face same direction as front'''
            for p, v in dict_reachable.items():
                ''' 1. dist to target (initiallizing the dict) '''
                dict_cost[str(v)] = [self.rate_dist(p)]
            
            vels, cost = sorted(dict_cost.items(), key=lambda item: item[1][0], reverse=False)[0]  
#            print(sorted(dict_cost.items(), key=lambda item: item[1][0], reverse=False)[0]  )
#            print(vels)
            return eval(vels)
        
        else :
            dist2front = self.dist2pose(self.front_pose, self.cur_pose)
            ''' facing toward front car '''
            ang_vel = self.angularVel(self.front_pose)
            ''' go straight to 1m away from front vehicle if there is no available vels '''
            lin_vel = self.linearVel(self.front_pose)

            if abs(dist2front - self.L) < self.DIST_ERR:
                lin_vel = 0.0

            elif dist2front < self.L:
                lin_vel = -lin_vel

            return [lin_vel, ang_vel]



if __name__ == '__main__':

    try:
        solamr_2 = LinkedDrive()
        rospy.init_node('linked_drive')
        rate = rospy.Rate(solamr_2.RATE)
        while not rospy.is_shutdown():

            ros_t0 = rospy.get_time()

#            ''' get predicted pose '''
#            front_predict_pose = solamr_2.pose_update(solamr_2.front_pose, solamr_2.front_vel, solamr_2.front_th) #[x, y, th]
#            ''' get quaternions '''
#            fp_qua = t.quaternion_from_euler(0.0, 0.0, front_predict_pose[2])
#            fo_qua = t.quaternion_from_euler(0.0, 0.0, solamr_2.cur_th)
#            ''' get potential poses '''
#            potential_poses = solamr_2.get_potential_poses()
#            dict_reachable = solamr_2.vels_from_pose(potential_poses)
#            reachable_poses = dict_reachable.keys()
#            print(solamr_2.front_vel)
            ''' update follower pose '''
            optimized_vels = solamr_2.pose_optimization()

            _twist = Twist()    
            _twist.linear.x = optimized_vels[0]
            _twist.angular.z = optimized_vels[1]
            solamr_2.r_vel_pub.publish(_twist)

            ros_td = rospy.get_time() - ros_t0
            if ros_td > 0 :
                print("pub Hz = {0}".format(1.0/ros_td))

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
