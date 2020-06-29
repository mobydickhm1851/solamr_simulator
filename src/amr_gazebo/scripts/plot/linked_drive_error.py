#!/usr/bin/env python

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style  # make the graphs more appealing (changable)
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from math import sqrt
import numpy as np
import time


#--- Parameter Setting ---#
#style.use('seaborn-whitegrid')
style.use('bmh')

#--- Color Setting ---#

light_blue = '#103A90'
green_blue = '#87CEFA'
light_orange = '#EE5100'
light_yellow = '#EE9500'
apple_green = '#B2DF36'
black = '#000000'
#---------------------#

fig = plt.figure()
ax = plt.subplot(111)
#ax = plt.axes(xlim=(0, 4), ylim=(-2, 2))
x, y = 0, 0
scat_x = ax.scatter(x, y, s=10, vmin=0, vmax=1,
                    c='r', edgecolor="k", label="x pose")

front_pose = [0.0, 0.0] # [x, y]
cur_pose = [0.0, 0.0] # [x, y]

lst_error = []
t0 = time.time()

def dist2pose(pose1, pose2):
    return np.sqrt(sum((np.array(pose1) - np.array(pose2))**2))

def f_pose_update(data):
    global front_pose
    ''' update x, y coordinate '''
    front_pose[0] = data.pose.pose.position.x
    front_pose[1] = data.pose.pose.position.y

def r_pose_update(data):
    global cur_pose
    ''' update x, y coordinate '''
    cur_pose[0] = data.pose.pose.position.x
    cur_pose[1] = data.pose.pose.position.y

def init():
    ax.clear()
    x, y = 0, 0
    scat_x = ax.scatter(x, y, s=10, vmin=0, vmax=1,
                                c='r', edgecolor="k", label="x pose")
#    ax.set_xlim([None, None])
    ax.set_title("Estimated Linked Drive Error", fontsize=18)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance between SOLamrs (m)')
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 2])
    return scat_x,

def animate(i):
    global lst_error, scar_x
    x = time.time() - t0
    y = dist2pose(front_pose, cur_pose)
    lst_error.append([x, y])
    scat_x.set_offsets(lst_error)
    return scat_x,

''' ROS node '''
rospy.init_node("Plot_Error", anonymous=True)
f_sub_1 = rospy.Subscriber("/solamr_1/odom", Odometry, f_pose_update)
r_sub_1 = rospy.Subscriber("/solamr_2/odom", Odometry, r_pose_update)

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, blit=True)

plt.show()

#anim.save('sine_wave.gif', writer='imagemagick')



