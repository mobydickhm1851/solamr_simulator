#! /usr/bin/env python

from math import atan2, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np
from string import uppercase as str_upp
# A set of point will be sented in and this modules will have the function 
# to find which groups they belong
# NOTE: now the set of points collected are those within the circle centered
# at shelft pose that ArUco found (this is cheating).

MAX_GP_DIST = 0.02 # Max dist allowed to count as a group
MIN_GP_N = 5 # Min number of scans in a group (to be counted as one)

def dist(lst0, lst1):
    ''' calculate the distance between two point '''
    return sqrt((lst0[0]-lst1[0])**2 + (lst0[1]-lst1[1])**2)

def newGroup(ID, in_dict, in_scan):
    ''' create a new group into the dict '''
    in_dict[str_upp[ID]] = np.zeros((0,2))
    in_dict[str_upp[ID]] = np.append(in_dict[str_upp[ID]], [in_scan], 0)
    return ID + 1 # Next group name 

def add2Group(key, in_dict, in_scan):
    ''' add scan to an existing group in dict '''
    in_dict[key] = np.append(in_dict[key], [in_scan], 0)

def belong2Group(in_scan, in_dict):
    ''' find the closest group the in_scan belongs to if it existed '''
    # Potential BUG : one scan can belong to two or more groups, 
    # but here we only return the group in_scan is the closest to
    belong_gp_key = ''
    MIN_DIST = 10.0 # random num that > MAX_GP_DIST
    for key in in_dict:
        for s in in_dict[key]:
            d = dist(in_scan, s)
            if  d <= MAX_GP_DIST and d < MIN_DIST and key not in belong_gp_key:
                belong_gp_key = key
                MIN_DIST = d
    if len(belong_gp_key) > 0 : return belong_gp_key
    else : return None

def minMember(in_dict):
    ''' delete the groups with member scans number < MIN_GP_N '''
    for key in in_dict.keys():
        if len(in_dict[key]) < MIN_GP_N : in_dict.pop(key, None)

def groupScan(in_scan, max_d=MAX_GP_DIST, min_n=MIN_GP_N):
    ''' main function to group incoming scans '''
    global MAX_GP_DIST, MIN_GP_N
    MAX_GP_DIST = max_d
    MIN_GP_N = min_n

    gp_dict = dict()
    GP_ID = 0 # using string.uppercase[:]
    
    for i in range(len(in_scan)):
        # First scan, init a group
        if len(gp_dict) == 0 : 
            GP_ID = newGroup(GP_ID, gp_dict, in_scan[i])
        # Belong to an existing group (dist to a scan < MAX_GP_DIST)
        elif belong2Group(in_scan[i], gp_dict) != None: 
            GP_KEY = belong2Group(in_scan[i], gp_dict)
            add2Group(GP_KEY, gp_dict, in_scan[i])
        else : 
            GP_ID = newGroup(GP_ID, gp_dict, in_scan[i])
    return gp_dict


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    n= 50
    scans = np.random.rand(n,2)
    
    fig = plt.figure()
    plt.scatter(scans[:,0], scans[:,1], s=20, vmin=0, vmax=1, c='None', edgecolor="r", label="scan")

    gp_d = groupScan(scans, max_d = 0.14)
    minMember(gp_d)
    for key in gp_d.keys():
        print(len(gp_d[key]))
        plt.scatter(gp_d[key][:,0], gp_d[key][:,1], s=50, vmin=0, vmax=1, c=np.random.rand(3,), edgecolor="k", label=key)

        
    try:
        plt.show()
        
    except rospy.ROSInterruptException:
        pass
