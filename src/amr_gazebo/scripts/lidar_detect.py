#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf.transformations as t
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from math import atan2, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np
# matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import matplotlib.patches as patches # show patches

style.use('bmh')

class LidarDetect:
    
    def __init__(self):

        rospy.init_node("position_follower", anonymous=True)
        #self.robot_ns = rospy.get_param('/solamr_1_AC/robot_ns') 
        self.robot_ns = "solamr_1"

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.lidar_sub = rospy.Subscriber("/{0}/scan_front".format(self.robot_ns),
                        LaserScan, self.lidarUpdate, ("scan_front"), queue_size=10)
        self.odom_sub = rospy.Subscriber("/{0}/odom".format(self.robot_ns),
                        Odometry, self.odomUpdate)
        self.shelft_pose_sub = rospy.Subscriber("/shelft_pose", Float64MultiArray, self.findObject, queue_size = 5)
        # data session
        self.pose_now = np.array([[0.0, 0.0]])
        self.theta = 0.0
        self.scan_xy = np.zeros((0,2))
        self.obj_dict = dict()
        self.range_max = 4.1
        self.map_size = 10
        self.shelft_l = .45
        self.shelft_w = .45

        # plot session
        self.fig = plt.figure(1, figsize=(18,8))
        self.ax0 = plt.subplot(121)
        self.ax1 = plt.subplot(122)
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=1, 
                 init_func=self.setupPlot, frames=360, blit=True)
	# Create a Rectangle patch
	self.rect0 = patches.Rectangle((self.pose_now[0]),0.4,0.4,linewidth=1,edgecolor='r',facecolor='none')
	self.rect1 = patches.Rectangle((self.pose_now[0]),0.4,0.4,linewidth=1,edgecolor='r',facecolor='none')

    def dist(self, lst0, lst1):
        ''' calculate the distance between two point '''
        return sqrt((lst0[0]-lst1[0])**2 + (lst0[1]-lst1[1])**2)

    def getRealPose(self, lidar_frame):
        ''' transition and rotation needed to turn scan data from relative to absolute '''
        r = rospy.Rate(10)
        try:
            ''' get the tf transform from lidar to base '''
            tf2 = self.tfBuffer.lookup_transform(
                    "{0}/base_footprint".format(self.robot_ns), 
                    "{0}/{1}".format(self.robot_ns, lidar_frame), rospy.Time())
            trans = tf2.transform.translation
            rot = tf2.transform.rotation #only x, y is needed
            (roll, pitch, lidar_theta) = t.euler_from_quaternion(
                                    [rot.x, rot.y, rot.z, rot.w])
            ''' from atan2 and 0 in x-dir, to rad and 0 in -y-dir '''
            return [lidar_theta, trans.x, trans.y]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
            r.sleep()
            return [0, 0, 0]

    def odomUpdate(self, msg):
        ''' Update current car pose ... '''
        self.pose_now[0][0] = msg.pose.pose.position.x
        self.pose_now[0][1] = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = t.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def lidarUpdate(self, scan, lidar_frame):
        ''' turn scan deg and range into pose in world frame '''
        ang_min = scan.angle_min
        ang_max = scan.angle_max
        ang_incr = scan.angle_increment
        self.range_max = scan.range_max # too_late for boundaries
        ''' refresh the lidar pose'''
        self.scan_xy = np.zeros((0,2))
        [rel_th, rel_x, rel_y] = self.getRealPose(lidar_frame)
        ''' turn translation from lidar to car to world frame '''
        b_th = self.theta + PI/2
        rot_mat = np.array([[np.cos(b_th), -np.sin(b_th)]
                ,[np.sin(b_th), np.cos(b_th)]])
        [w_x, w_y] = np.dot(rot_mat, [rel_x, rel_y]) + self.pose_now[0] 

        for i in range(len(scan.ranges)):
            ang_i = ang_min + ang_incr*(i+1) + (rel_th + self.theta) 
            xy = [scan.ranges[i]*np.cos(ang_i) + w_x,
                scan.ranges[i]*np.sin(ang_i) + w_y ]
            self.scan_xy = np.append(self.scan_xy, [xy], 0)

        #print("min={0}, max={1}, ang_increment={2}".format(ang_min, ang_max, ang_incr))
        #print(np.shape(scan.ranges))

    def findObject(self, data):
        ''' find possible scans with the pose found using ArUco '''
        ''' data in Float64MultiArray [car_x, car_y, shelft_x, shelft_y] '''
        data_name = data.layout.dim[0].label.split(':')[-1]
        ''' refresh the object pose '''
        self.obj_dict[data_name]=np.zeros((0,2))
        aruco_pose = data.data[2:4]
        dist2shelft = sqrt((self.shelft_w+0.1)**2 + (self.shelft_l+0.1)**2)
        #print(dist2shelft)
        for scan in self.scan_xy:
            if self.dist(scan, aruco_pose) <= dist2shelft :
                self.obj_dict[data_name]= np.append(self.obj_dict[data_name], [scan], 0)
                #print(self.obj_dict)


    def setupPlot(self):
        """Initial drawing of the scatter plot."""
        dark_orange = np.array([ 0.90588,  0.45882,  0.01176,  1.])
        x, y = 0, 0
        self.ax0.clear()
        self.ax1.clear()
        self.scat_xy_0 = self.ax0.scatter(x, y, s=20, vmin=0, vmax=1,
                                        c='None', edgecolor="r", label="scan")
        self.scat_xy_1 = self.ax1.scatter(x, y, s=20, vmin=0, vmax=1,
                                        c='None', edgecolor="r", label="scan")
        self.scat_obj_0 = self.ax0.scatter(x, y, s=30, vmin=0, vmax=1,
                                        c='b', marker="+", label="object")
        self.scat_obj_1 = self.ax1.scatter(x, y, s=30, vmin=0, vmax=1,
                                        c='b', marker="+", label="object")
        self.ax0.axis([-self.map_size, self.map_size,
                        -self.map_size, self.map_size])
        self.ax1.axis([-self.range_max + self.pose_now[0][0]+0.5,
                        self.range_max + self.pose_now[0][0]+0.5,
                        -self.range_max + self.pose_now[0][1]+0.5,
                        self.range_max + self.pose_now[0][1]+0.5])
        self.ax0.set_title("Lidar Plot [Global Frame]", fontsize=18)
        self.ax1.set_title("Lidar Plot [Local Frame]", fontsize=18)
        self.ax0.legend(scatterpoints = 1, loc='upper left')
        self.ax1.legend(scatterpoints = 1, loc='upper left')
        self.ax0.set_xlabel('X-axis (m)')
        self.ax0.set_ylabel('Y-axis (m)')
        self.ax1.set_xlabel('x-axis (m)')
        self.ax1.set_ylabel('y-axis (m)')
	# Create a Rectangle patch
	self.ax0.add_patch(self.rect0)
	self.ax1.add_patch(self.rect1)
        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_xy_0, self.scat_obj_0, self.scat_xy_1, self.scat_obj_1, self.rect0, self.rect1,

    def update(self, i):
        """Update the scatter plot."""
        # Set x and y data... Nx2 n-D array
        self.scat_xy_0.set_offsets(self.scan_xy)
        self.scat_xy_1.set_offsets(self.scan_xy)
        if len(self.obj_dict)>0:
            self.scat_obj_0.set_offsets(self.obj_dict['202'])
            self.scat_obj_1.set_offsets(self.obj_dict['202'])

        self.ax1.set_xlim(-self.range_max + self.pose_now[0][0]-0.5,
                        self.range_max + self.pose_now[0][0]+0.5)
        self.ax1.set_ylim(-self.range_max + self.pose_now[0][1]-0.5,
                        self.range_max + self.pose_now[0][1]+0.5)
	# Create a Rectangle patch
	self.rect0.set_xy(self.pose_now[0])
	self.rect1.set_xy(self.pose_now[0])
        # Set sizes...
        #self.scat.set_sizes(300 * abs(data[:, 2])**1.5 + 100)
        # Set colors..
        #self.scat.set_array(data[:, 3])

        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_xy_0, self.scat_obj_0, self.scat_xy_1, self.scat_obj_1, self.rect0, self.rect1,

if __name__ == '__main__':
    
    try:
        lidar = LidarDetect()
        plt.show()
        while not rospy.is_shutdown():
            rospy.spin()
            #solamr0.move2Goal(2.04,1.0,0)

    except rospy.ROSInterruptException:
        pass
