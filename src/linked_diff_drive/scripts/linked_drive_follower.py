#! /usr/bin/env python 

# Generate random path for diff drive
from __future__ import print_function
from math import atan2, exp, sqrt, log
from math import pi as PI
import numpy as np
import copy
import math

import rospy
import tf.transformations as t
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
#from std_msgs.msg import Header


class MarkerPub:
    def __init__(self, name, marker_type, rgba, scale=[0.1,0.1,0.1]):
        ''' MARKERS '''
        self.name = name # str
        self.marker_type = marker_type # int
#        if self.marker_type == 2: self.scale = [0.1, 0.1, 0.1] # SPHERE
#        elif self.marker_type == 1: self.scale = [0.05, 0.05, 0.05] # CUBE
#        elif self.marker_type == 0: self.scale = [0.5, 0.03, 0.03] # ARROW
#        else: self.scale = [0.06, 0.06, 0.06] # OTHERS
        self.rgba = rgba # list
        self.scale = scale
        self.array = MarkerArray() 
        ''' COUNTS '''
        self.COUNT = 0
        self.MARKER_MAX = 1
        ''' ROS node '''
        self.marker_pub = rospy.Publisher(self.name, MarkerArray, queue_size=10)

    def publish_marker(self, poses, quaternion=[0,0,0,1.0], clear=False):
        ''' erase all and re-append '''

        self.array = MarkerArray()

        for pose in poses:

            m = Marker()
            m.lifetime = rospy.Duration.from_sec(0.1)
            m.header.frame_id = 'map'
            m.type = self.marker_type
            m.action = m.ADD
            m.scale.x = self.scale[0]
            m.scale.y = self.scale[1]
            m.scale.z = self.scale[2]
            m.color.r = self.rgba[0] 
            m.color.g = self.rgba[1] 
            m.color.b = self.rgba[2] 
            m.color.a = self.rgba[3]
            m.pose.orientation.x = quaternion[0] 
            m.pose.orientation.y = quaternion[1] 
            m.pose.orientation.z = quaternion[2] 
            m.pose.orientation.w = quaternion[3] 
            m.pose.position.x = pose[0] 
            m.pose.position.y = pose[1]
            m.pose.position.z = 0.0 

            self.array.markers.append(m)
            
#            ''' add new marker and remove the old one '''
#            self.MARKER_MAX = len(poses)
#            if self.COUNT > self.MARKER_MAX:
#                self.array.markers.pop(0)
#            self.array.markers.append(m)
#            self.COUNT += 1 

        ''' Renumber the marker IDs '''
        id = 0
        for ma in self.array.markers:
            ma.id = id
            id += 1

        self.marker_pub.publish(self.array)

    def marker_transfer(self, x, y, child, parent="map"):

        time = rospy.Time.now()
        self.br.sendTransform( (x, y, 0), (0, 0, 0, 1.0), time, parent, child )
        

class LinkedDrive:

    def __init__(self, rot_vel_max=2.5718, # In rad/s
                       lin_vel_max=3.0, # In m/s
                       rot_acc=2.0, lin_acc=2.0): # in rad/s^2 and m/s^2

        ''' Optimization Ratios '''
        self.RATIO_DIST = 1.0

        ''' Shared params '''
        self.dt = 0.3 # sec
        self.L = 1.0 # dist between two solamr when connected with shelft
        self.ANG_RES = 0.006

        ''' variables of FRONT car '''
        self.RATE = 10
        self.front_pose = [0.0, 0.0] # [x, y]
        self.front_ori = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
        self.front_th = 0.0
        self.front_vel = [0.0, 0.0]  # [v_linear, w_angular]
        
        ''' variables and params of REAR car '''
        self.INIT_X = - self.L
        self.INIT_Y = 0.0
        self.cur_pose = [self.INIT_X, self.INIT_Y] # [x, y]
        self.cur_vel = [0.0, 0.0] # [lin, ang]
        self.cur_th = 0.0
        self.ROT_VEL = [-rot_vel_max, rot_vel_max]
        self.LIN_VEL = [-lin_vel_max, lin_vel_max]
        self.ROT_ACC = [-rot_acc, rot_acc]
        self.LIN_ACC = [-lin_acc, lin_acc]

        ''' ROS node '''
        self.f_sub_1 = rospy.Subscriber("/front_pose", PoseStamped, self.f_pose_update)
        self.f_sub_2 = rospy.Subscriber("/front_twist", Twist, self.f_twist_update)

    def f_pose_update(self, data):
        ''' update x, y coordinate '''
        self.front_pose[0] = data.pose.position.x
        self.front_pose[1] = data.pose.position.y
        
        ''' update quaternion '''
        self.front_ori[0] = data.pose.orientation.x
        self.front_ori[1] = data.pose.orientation.y
        self.front_ori[2] = data.pose.orientation.z
        self.front_ori[3] = data.pose.orientation.w

        (row, pitch, self.front_th) = t.euler_from_quaternion(self.front_ori)

    def f_twist_update(self, data):
        self.front_vel[0] = data.linear.x
        self.front_vel[1] = data.angular.z


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
        [x, y] = self.pose_update(self.front_pose, self.front_vel, self.front_th)[:2]
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
            if self.check_vels_range([v, w]) and self.dist2pose(temp_pose, pose) < 0.0001: 
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
        

    def follower_update(self, vel):
        [x1, y1, th1] = self.pose_update(self.cur_pose, vel, self.cur_th)
        self.cur_pose = [x1, y1]
        self.cur_th = th1
        self.cur_vel = vel

    def dist2pose(self, pose1, pose2):
        return np.sqrt(sum((np.array(pose1) - np.array(pose2))**2))
        
    def pointAngularDiff(self, goal):
        x_diff = goal[0] - self.cur_pose[0]
        y_diff = goal[1] - self.cur_pose[1]
        theta_goal = atan2(y_diff, x_diff)
        return  (theta_goal - self.cur_th)

    def angularVel(self, point, RATE=0.5, backward=True):
        THETA_TOL = 0.0175  # in radian ~= 2 deg
        MAX_OMEGA = self.ROT_VEL[1]
        MIN_OMEGA = 0.1
        theta_diff = self.pointAngularDiff(point)
        ang_temp = 0.0000000001
        ''' prevent oscilliation '''
        if abs(theta_diff) < THETA_TOL*2 : RATE*=0.3
        elif abs(theta_diff) < THETA_TOL*11 : RATE*=0.5
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
        if abs(ang_temp) >= MAX_OMEGA: ang_temp = MAX_OMEGA * abs(ang_temp)/ang_temp
#        elif abs(ang_temp) <= MIN_OMEGA: ang_temp = MIN_OMEGA * abs(ang_temp)/ang_temp
        return ang_temp

    def rate_dist(self, target_pose):
        ''' rate the distance between front and follower (closer the lower) '''
        r_d = self.RATIO_DIST
        follower_pose = self.cur_pose
        return r_d * self.dist2pose(follower_pose, target_pose)

    def rate_ori(self, target_pose, mode="shelft"):
        ''' rate the orientation, standatd: 1.shelft orientation, 2.front car orientation '''

        ''' 1.shelft orientation as standard '''

        ''' 2.front car orientation as standard '''

        pass
    
    def pose_optimization(self):
        ''' return optimized pose from reachable poses '''
        potential_poses = self.get_potential_poses()
        dict_reachable = self.vels_from_pose(potential_poses)
        reachable_poses = dict_reachable.keys()
        print(reachable_poses)

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
            ''' facing toward front car '''
            ang_vel = self.angularVel(self.front_pose)
            print(ang_vel)
#            print("No rechable pose exists.")
            return [0.0, ang_vel]




    def main(self):

        rospy.init_node('linked_drive')
        rate = rospy.Rate(self.RATE)
        while not rospy.is_shutdown():

            ros_t0 = rospy.get_time()

            ''' Initialize markers '''
            front = MarkerPub(name='front_marker', marker_type=2, 
                    rgba=[1.0,1.0,1.0,1.0], scale=[0.1, 0.1, 0.1])
            front_predict = MarkerPub(name='front_predict_marker', marker_type=2, 
                    rgba=[0.0,1.0,0.0,0.3])
            front_arrow = MarkerPub(name='front_arrow', marker_type=0, 
                    rgba=[1.0,1.0,1.0,1.0], scale=[0.5, 0.03, 0.03])
            front_predict_arrow = MarkerPub(name='front_predict_arrow', marker_type=0, 
                    rgba=[0.0,1.0,0.0,0.3], scale=[0.5, 0.03, 0.03])
            follower = MarkerPub(name='follower_marker', marker_type=2, 
                    rgba=[1.0,0.0,0.0,1.0], scale=[0.1, 0.1, 0.1])
            follower_arrow = MarkerPub(name='follower_arrow', marker_type=0, 
                    rgba=[1.0,0.0,0.0,1.0], scale=[0.5, 0.03, 0.03])
#            predict_potential = MarkerPub(name='potential_poses', marker_type=1, 
#                    rgba=[0.5,0.95,0.8,0.4], scale=[0.05, 0.05, 0.05])
            reachable = MarkerPub(name='reachable_poses', marker_type=1, 
                    rgba=[1.0,0.0,0.0,1.0], scale=[0.05, 0.05, 0.05])
            body = MarkerPub(name='body_marker', marker_type=0, 
                    rgba=[.8,.8,.8,1.0], scale=[1.0, 0.2, 0.01])
            ''' get predicted pose '''
            front_predict_pose = self.pose_update(self.front_pose, self.front_vel, self.front_th) #[x, y, th]
            ''' get quaternions '''
            fp_qua = t.quaternion_from_euler(0.0, 0.0, front_predict_pose[2])
            fo_qua = t.quaternion_from_euler(0.0, 0.0, self.cur_th)
            ''' get potential poses '''
            potential_poses = self.get_potential_poses()
            dict_reachable = self.vels_from_pose(potential_poses)
            reachable_poses = dict_reachable.keys()
#            print(reachable_poses)
#            print(self.front_vel)
            ''' update follower pose '''
            optimized_vels = self.pose_optimization()
            self.follower_update(optimized_vels)
            ''' Publish markers (array) '''
            front.publish_marker([self.front_pose])
            front_arrow.publish_marker([self.front_pose], self.front_ori)
            front_predict.publish_marker([front_predict_pose])
            front_predict_arrow.publish_marker([front_predict_pose], fp_qua)
            follower.publish_marker([self.cur_pose])
            follower_arrow.publish_marker([self.cur_pose], fo_qua)
#            predict_potential.publish_marker(potential_poses)
            if len(reachable_poses) > 0: 
                reachable.publish_marker(reachable_poses, clear=True)
            ''' publish imaginary shelft body '''
            dx = front_predict_pose[0] - self.cur_pose[0]
            dy = front_predict_pose[1] - self.cur_pose[1]
            body_qua = t.quaternion_from_euler(0.0, 0.0, np.arctan2(dy, dx))
            body.publish_marker([self.cur_pose], body_qua)

            ros_td = rospy.get_time() - ros_t0
            print("pub Hz = {0}".format(1.0/ros_td))
            
            rate.sleep()

if __name__ == '__main__':

    try:
        test = LinkedDrive()
        test.main()

    except rospy.ROSInterruptException:
        pass
