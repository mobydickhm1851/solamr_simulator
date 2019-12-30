#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, TransformStamped
from math import atan2, exp, sqrt, log
from math import pi as PI
import copy
import numpy as np

class arucotf2Broadcast:
    
    def __init__(self):

        rospy.init_node("aruco_tf2_broadcast", anonymous=True)
        self.fiducial_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.tf2Broadcast)

    def tf2Broadcast(self, msg):
        imageTime = msg.header.stamp
        print (imageTime, rospy.Time.now())
        print ("*****")
        br = tf2_ros.TransformBroadcaster() 
        # For every fiducial found by the dectector, publish a transform
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            print "Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                             (id, trans.x, trans.y, trans.z,
                               rot.x, rot.y, rot.z, rot.w)
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            t.header.frame_id = msg.header.frame_id
            t.header.stamp = imageTime
            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w
            br.sendTransform(t)

    def run(self):
        
        return


if __name__ == '__main__':
    
    while not rospy.is_shutdown(): 
        try:
            Br1 = arucotf2Broadcast()
            rospy.spin()


        except rospy.ROSInterruptException:
            pass
