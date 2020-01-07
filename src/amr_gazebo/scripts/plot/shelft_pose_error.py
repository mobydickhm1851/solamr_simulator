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
from scipy.stats import norm


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

class PlotTopic:

    def __init__(self):
        ''' rosnode init and publisher and subscriber '''
        rospy.init_node("PlotTopic", anonymous=True)
        self.dist_error_sub = rospy.Subscriber('/shelft_pose', 
                            Float64MultiArray, self.data_update)
        ''' list and variables '''
        self.target_pose = [2.0, 1.0] # pose of the shelft
        self.shelft_error_dict = dict()
        self.est_x_dict = dict()
        self.est_y_dict = dict()
        #self.dist_error_arr =np.zeros((0,2)) #init [car_x,car_y,shelft_x,shelft_y]
        ''' plt, animation'''
        self.fig = plt.figure(1, figsize=(12,8))
        self.ax = plt.subplot(211)
        self.ax0 = plt.subplot(223)
        self.ax1 = plt.subplot(224)

        self.ani = animation.FuncAnimation(self.fig, self.update, interval=5, 
                 init_func=self.setup_plot, blit=True)


    def error_estimate(self, lst, mod='e'):
        ''' 'x': saving the estimated x error w.r.t. dist2goal '''
        ''' 'y': saving the estimated y error w.r.t. dist2goal '''
        ''' 'e': calculate the error of estimation '''
        dist2goal = sqrt((lst[0] - self.target_pose[0])**2 
                  + (lst[1] - self.target_pose[1])**2 )
        error = sqrt((lst[2] - self.target_pose[0])**2 
                  + (lst[3] - self.target_pose[1])**2 )
        x_er = sqrt((lst[2] - self.target_pose[0])**2)
        y_er = sqrt((lst[3] - self.target_pose[1])**2)
        if mod=='e':  return [dist2goal, error]
        elif mod=='x': return [dist2goal, x_er]
        elif mod=='y': return [dist2goal, y_er]

    def data_update(self, data):
        ''' data in Float64MultiArray [car_x, car_y, shelft_x, shelft_y] '''
        data_name = data.layout.dim[0].label.split(':')[-1]
        if len(self.shelft_error_dict) == 0 or not data_name in self.shelft_error_dict.keys():
            self.shelft_error_dict[data_name]=np.zeros((0,2))
            self.est_x_dict[data_name]=np.zeros((0,2))
            self.est_y_dict[data_name]=np.zeros((0,2))
        dist_error = self.error_estimate(data.data, 'e')
        self.shelft_error_dict[data_name] = np.append(self.shelft_error_dict[data_name], [dist_error], 0)
        dist_x = self.error_estimate(data.data, 'x')
        self.est_x_dict[data_name] = np.append(self.est_x_dict[data_name], [dist_x], 0)
        dist_y = self.error_estimate(data.data, 'y')
        self.est_y_dict[data_name] = np.append(self.est_y_dict[data_name], [dist_y], 0)

    def setup_plot(self):
        """Initial drawing of the scatter plot."""
        # This init function is actually called twice : setup the animation
        # internally and then in the code, so clear everything first or there
        # might be something wrong like double-legend
        self.ax.clear()
        self.ax0.clear()
        self.ax1.clear()
        dark_orange = np.array([ 0.90588,  0.45882,  0.01176,  1.])
        x, y = 0, 0
        self.scat_x = self.ax.scatter(x, y, s=10, vmin=0, vmax=1,
                                    c='r', edgecolor="k", label="x pose")
        self.scat_y = self.ax.scatter(x, y, s=10, vmin=0, vmax=1,
                                    c='b', edgecolor="k", label="y pose")
        self.scat0 = self.ax0.scatter(x, y, s=10, vmin=0, vmax=1,
                                    c=dark_orange, edgecolor="k")
        self.scat1 = self.ax1.scatter(x, y, s=10, vmin=0, vmax=1,
                                    c=dark_orange, edgecolor="k")
        self.ax.axis([0, 10, 0, 2])
        self.ax0.axis([0, 10, 0, 1])
        self.ax1.axis([0, 5, 0, .25])
        self.ax.set_title("Estimated Pose w.r.t. Distance to Goal", fontsize=18)
        self.ax0.set_title("Estimation Error w.r.t. Distance to Goal", fontsize=12)
        self.ax1.set_title("Estimation Error w.r.t. Distance to Goal", fontsize=12)
        self.ax.legend(scatterpoints = 1, loc='upper left')
        self.ax.set_xlabel('Distance to Goal (m)')
        self.ax.set_ylabel('Estimation (m)')
        self.ax0.set_xlabel('Distance to Goal (m)')
        self.ax0.set_ylabel('Estimation Error (m)')
        self.ax1.set_xlabel('Distance to Goal (m)')
        self.ax1.set_ylabel('Estimation Error (m)')
        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_x, self.scat_y, self.scat0, self.scat1,

    def update(self, i):
        """Update the scatter plot."""
        if len(self.shelft_error_dict) != 0 and "202" in self.shelft_error_dict.keys():
            # Set x and y data... Nx2 n-D array
            self.scat_x.set_offsets(self.est_x_dict["202"])
            self.scat_y.set_offsets(self.est_y_dict["202"])
            self.scat0.set_offsets(self.shelft_error_dict["202"])
            self.scat1.set_offsets(self.shelft_error_dict["202"])
            # Set sizes...
            #self.scat.set_sizes(300 * abs(data[:, 2])**1.5 + 100)
            # Set colors..
            #self.scat.set_array(data[:, 3])

            # We need to return the updated artist for FuncAnimation to draw..
            # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat_x, self.scat_y, self.scat0, self.scat1,


if __name__ == '__main__':

    shelft_pose = PlotTopic()
    plt.show()

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass



