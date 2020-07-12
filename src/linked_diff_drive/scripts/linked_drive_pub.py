#! /usr/bin/env python

from math import atan2, exp, sqrt, log
from math import pi as PI
import numpy as np
import copy
import time

import rospy
import tf.transformations as t
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class LinkedDrive:
    """
    Description:

    Attributes:
        ratio_dist (float): Optimization Ratios
        dt (float):
        L (float): distance between two solamr when connected with shelft
        dist_error (float):
        angle_resolution (float):
        rot_vel_max (float):
        lin_vel_max (float):
        rot_acc (float):
        lin_acc (float):
    """
    def __init__(self):
        self.ratio_dist = 1.0
        self.dt = 0.3  # sec
        self.L = 1.0  # m
        self.dist_error = 0.05
        self.angle_resolution = 0.01
        self.rot_vel_max = 3.5718  # rad/s
        self.lin_vel_max = 5.0  # m/s
        self.rot_acc = 3.0  # rad/s^2
        self.lin_acc = 3.0  # m/s^2

    def get_predict_pose(self, pose, vel, th0):
        '''
        Description:
            prediction of trajectory using arc (combination of v and w)
        Args:
            pose (list):
            vel (list):
            th0 (float):
        Return:
            (tuple): x1, y1, th1
        '''
        x0 = pose[0]
        y0 = pose[1]
        v = vel[0]
        w = vel[1]

        if abs(w) < 1e-5:
            dx = v * self.dt
            dy = 0.0
            dth = 0.0
        else:
            r = abs(v / w)
            dth = w * self.dt
            dx = abs(r * np.sin(dth))
            dx = dx if v > 0 else -dx
            dy = r * (1.0 - np.cos(dth))

        rot_mat = np.array([[np.cos(th0), -np.sin(th0)], [np.sin(th0), np.cos(th0)]])
        [[x1], [y1]] = [[x0], [y0]] + np.dot(rot_mat, [[dx], [dy]])
        th1 = th0 + dth

        return x1, y1, th1

    def get_potential_poses(self):
        '''
        Description:
            get potential locations (poses) for follower to be at
        Return:
            (list):
        '''
        # -- get predicted front pose
        x, y, _ = self.get_predict_pose(FRONT_POSE, FRONT_VEL, FRONT_TH)  # from front's current vels
        # x, y, _ = self.get_predict_pose(FRONT_POSE, FRONT_CMD_VEL, FRONT_TH)  # from front's predicted vels

        list_rad = np.arange(0, 2.0*PI, self.angle_resolution) # rad, potential poses every __ rad
        # print("potential poses = {0}".format(len(lst_rad)))
        list_poses = []

        for th in list_rad:
            list_poses.append([x + self.L * np.cos(th), y + self.L * np.sin(th)])

        m_arr = get_marker_array(list_poses)
        PUB_MARKER_ARRAY.publish(m_arr)

        return list_poses

    def vels_from_pose(self, poses):
        '''
        Description:
            get lin/ang velocities from given pose
        Args:
            poses (list):
        Return:
            (dict):
        '''
        dict_reachable = dict()
        mat_rot_inv = np.array([[np.cos(CUR_TH), np.sin(CUR_TH)], [-np.sin(CUR_TH), np.cos(CUR_TH)]])

        for pose in poses:
            [[dx], [dy]] = np.dot(mat_rot_inv, np.array([[pose[0]-CUR_POSE[0]], [pose[1]-CUR_POSE[1]]]))

            if dy == 0: # only lin_vel, no rotation
                w = 0.0
                v = dx / self.dt
            else:
                r = (dx**2 + dy**2) / (2.0*dy)
                # -- w = omega and v = vel.x
                w = np.arcsin(dx / np.array(r)) / self.dt # 1x2 mat
                v = np.array(r) * np.array(w)

            # print("pose={0}; v and w={1}; check_result={2}; dxdy={3}".format(pose, [v,w], self.check_vels_range([v,w]), [dx,dy]))
            cur_x1, cur_y1, _ = self.get_predict_pose(CUR_POSE, (v,w), CUR_TH)

            if self.check_vels_range((v, w)) and get_dist_from_two_poses((cur_x1, cur_y1), pose) < 0.01:
                dict_reachable[tuple(pose)] = [v, w]

        return dict_reachable

    def check_vels_range(self, vels):
        '''
        Description:
            check if the [v, w] is within the bounds
        Args:
            vels (tuple): v, w
        Return:
            (bool):
        '''
        v1, w1 = vels
        v0, w0 = CUR_VEL
        av, aw = (np.array(vels) - np.array(CUR_VEL)) / self.dt

        if abs(v1) > self.lin_vel_max:
            return False
        if abs(w1) > self.rot_vel_max:
            return False
        if abs(av) > self.lin_acc:
            return False
        if abs(aw) > self.rot_acc:
            return False

        return True

    def pointAngularDiff(self, goal):
        '''
        Description:

        Args:
            goal (list):
        Return:
            (float)
        '''
        x_diff = goal[0] - CUR_POSE[0]
        y_diff = goal[1] - CUR_POSE[1]
        theta_goal = atan2(y_diff, x_diff)
        return theta_goal - CUR_TH

    def angularVel(self, point, rate_ang=0.5, backward=True):
        '''
        Description:

        Args:
            point (?):
            rate_ang (float):
            backward (bool):
        Return:
            (float):
        '''
        theta_tol = 0.0175  # in radian ~= 2 deg
        max_omega = self.rot_vel_max
        min_omega = 0.1
        theta_diff = self.pointAngularDiff(point)

        # -- TEST: see if this can follow better
        # theta_diff -= PI/8
        ang_temp = 0.0000000001

        # -- prevent oscilliation
        if abs(theta_diff) < theta_tol*2:
            rate_ang *= 0.3
        elif abs(theta_diff) < theta_tol*11:
            rate_ang*=0.5
        else:
            pass

        # -- turn CW or CCW
        if theta_diff > 0:
            if theta_diff > PI:
                ang_temp =  - rate_ang * exp(2*PI - theta_diff)
            else :
                ang_temp =  rate_ang * exp(theta_diff)

        if theta_diff < 0:
            if abs(theta_diff) > PI:
                ang_temp = rate_ang * exp(2*PI + theta_diff)
            else :
                ang_temp = - rate_ang * exp(- theta_diff)

        if abs(ang_temp) >= max_omega:
            ang_temp = max_omega * abs(ang_temp)/ang_temp
        # elif abs(ang_temp) <= min_omega:
        #     ang_temp = min_omega * abs(ang_temp)/ang_temp
        # else:
        #     pass

        return ang_temp

    def faceSameDir(self, goal):
        '''
        Description:
            Decide to drive forward or backward
        Args:
            goal (?):
        Return:
            (bool):
        '''
        if abs(self.pointAngularDiff(goal)) < PI/2.0 or abs(self.pointAngularDiff(goal)) > PI*3.0/2.0 :
            return True # same dir, drive forward
        else:
            return False # opposite dir, drive reverse

    def linearVel(self, goal, rate_lin=0.5):
        '''
        Description:

        Args:
            goal (?):
            rate_lin (float):
        Return:
            (float):
        '''
        dist = get_dist_from_two_poses(CUR_POSE, goal)

        if self.faceSameDir(goal) :
            vel_temp = rate_lin * log(dist+1)
        elif not self.faceSameDir(goal) :
            vel_temp = - rate_lin * log(dist+1)
        # -- MIN and MAX
        if abs(vel_temp) >= self.lin_vel_max:
            vel_temp = self.lin_vel_max * abs(vel_temp)/vel_temp

        return vel_temp

    def rate_dist(self, target_pose):
        '''
        Description:
            rate the distance between front and follower (closer the lower)
        Args:
            target_pose (?):
        Return:
            (float):
        '''
        r_d = self.ratio_dist
        follower_pose = CUR_POSE
        return r_d * get_dist_from_two_poses(follower_pose, target_pose)

    def rate_ori(self, target_pose, mode="shelft"):
        '''
        Description:
            rate the orientation, standatd: 1.shelft orientation, 2.front car orientation
        Args:
            target_pose (?):
            mode (str):
        Return:
            (float)
        '''
        # -- 1.rate by shelft orientation

        # -- 2.rate by front car orientation
        pass

    def get_opt_rear_vels(self):
        '''
        Description:
            get optimized linear and rotation velocity from reachable poses
        Return:
            (tuple):
        '''
        potential_poses = self.get_potential_poses()
        dict_reachable = self.vels_from_pose(potential_poses)
        reachable_poses = dict_reachable.keys()
        print(dict_reachable)

        if len(reachable_poses) > 0:
            dict_cost = dict()
            # -- optimization according to : dist to target, face same direction as front
            for p, v in dict_reachable.items():
                # -- 1. dist to target (initiallizing the dict)
                dict_cost[str(v)] = [self.rate_dist(p)]

            vels, cost = sorted(dict_cost.items(), key=lambda item: item[1][0], reverse=False)[0]
            # print(sorted(dict_cost.items(), key=lambda item: item[1][0], reverse=False)[0]  )
            # print(vels)
            return eval(vels)

        else :
            dist2front = get_dist_from_two_poses(FRONT_POSE, CUR_POSE)

            # -- facing toward front car
            ang_vel = self.angularVel(FRONT_POSE)

            # -- go straight to 1m away from front vehicle if there is no available vels
            lin_vel = self.linearVel(FRONT_POSE)

            if abs(dist2front - self.L) < self.dist_error:
                lin_vel = 0.0
            elif dist2front < self.L:
                lin_vel = -lin_vel
            else:
                pass

            return (lin_vel, ang_vel)


# --

def get_dist_from_two_poses(pose1, pose2):
    '''
    Description:
        get distance between two pose
    Args:
        pose1 (?):
        pose2 (?):
    Return:
        (float):
    '''
    return np.sqrt(sum((np.array(pose1) - np.array(pose2))**2))

def _cb_fp_cmdvel(data):
    """
    Descriptions:
        callback for rospy.Subscriber /solamr_1/cmd_vel
    """
    global FRONT_CMD_VEL

    FRONT_CMD_VEL[0] = data.linear.x
    FRONT_CMD_VEL[1] = data.angular.z

def _cb_f_pose(data):
    """
    Descriptions:
        callback for rospy.Subscriber /solamr_1/odom
    """
    global FRONT_POSE, FRONT_VEL, FRONT_ORI, FRONT_TH

    FRONT_POSE[0] = data.pose.pose.position.x
    FRONT_POSE[1] = data.pose.pose.position.y

    FRONT_VEL[0] = data.twist.twist.linear.x
    FRONT_VEL[1] = data.twist.twist.angular.z

    FRONT_ORI[0] = data.pose.pose.orientation.x
    FRONT_ORI[1] = data.pose.pose.orientation.y
    FRONT_ORI[2] = data.pose.pose.orientation.z
    FRONT_ORI[3] = data.pose.pose.orientation.w

    _, _, FRONT_TH = t.euler_from_quaternion(FRONT_ORI)

def _cb_r_pose(data):
    """
    Descriptions:
        callback for rospy.Subscriber /solamr_2/odom
    """
    global CUR_POSE, CUR_ORI, CUR_TH

    CUR_POSE[0] = data.pose.pose.position.x
    CUR_POSE[1] = data.pose.pose.position.y

    CUR_ORI[0] = data.pose.pose.orientation.x
    CUR_ORI[1] = data.pose.pose.orientation.y
    CUR_ORI[2] = data.pose.pose.orientation.z
    CUR_ORI[3] = data.pose.pose.orientation.w

    _, _, CUR_TH = t.euler_from_quaternion(CUR_ORI)

def get_marker_points():
    """
    Descritions:

    Return:
        (Marker):
    """
    m = Marker()
    m.header.frame_id = "/solamr_1/odom"
    m.header.stamp = rospy.Time.now()
    m.ns = "/"
    m.id = 0
    m.action = m.ADD
    m.type = m.POINTS
    m.pose.orientation.w = 1.0
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.color.g = 1.0
    m.color.a = 1.0
    return m

def get_marker_array(locs):
    """
    Descritions:

    Args:
        locs (list of tuple): [(x, y), ...]
    Return:
        (MarkerArray):
    """
    marker_array = MarkerArray()
    points = get_marker_points()
    for loc in locs:
        p = Point()
        p.x = loc[0]
        p.y = loc[1]
        p.z = 0.0
        points.points.append(p)
    marker_array.markers.append(points)
    return marker_array

if __name__ == '__main__':

    # -- global vars
    FRONT_CMD_VEL = [0.0, 0.0]  # [v_linear, w_angular]
    FRONT_POSE = [0.0, 0.0]  # [x, y]
    FRONT_VEL = [0.0, 0.0]  # [v_linear, w_angular]
    FRONT_ORI = [0.0, 0.0, 0.0, 0.0] # [x, y, z, w]
    FRONT_TH = 0.0

    CUR_POSE = [-1.0, 0.0]  # [x, y]
    CUR_VEL = [0.0, 0.0]  # [lin, ang]
    CUR_ORI = [0.0, 0.0, 0.0, 0.0]  # [x, y, z, w]
    CUR_TH = 0.0

    # -- rosnode
    rospy.init_node('linked_drive')

    rospy.Subscriber(name="/solamr_1/odom", data_class=Odometry, callback=_cb_f_pose)
    rospy.Subscriber(name="/solamr_1/cmd_vel", data_class=Twist, callback=_cb_fp_cmdvel)
    rospy.Subscriber(name="/solamr_2/odom", data_class=Odometry, callback=_cb_r_pose)
    R_VEL_PUB = rospy.Publisher(name='/solamr_2/cmd_vel', data_class=Twist, queue_size=10)
    PUB_MARKER_ARRAY = rospy.Publisher(name="visualization_marker_array", data_class=MarkerArray, queue_size=1)

    # -- linked_drive loop
    linked_drive = LinkedDrive()

    try:
        rate = rospy.Rate(hz=10)
        while not rospy.is_shutdown():
            ros_t0 = rospy.get_time()

            # -- update follower pose
            rear_vels = linked_drive.get_opt_rear_vels()

            _twist = Twist()
            _twist.linear.x = rear_vels[0]
            _twist.angular.z = rear_vels[1]

            R_VEL_PUB.publish(_twist)

            ros_td = rospy.get_time() - ros_t0

            if ros_td > 0 :
                print("pub Hz = {0}".format(1.0/ros_td))

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
