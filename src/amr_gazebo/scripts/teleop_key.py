#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Ini.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from math import pi as PI

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
Block for connection : t/b for CW/CCW
Switch between solamr 1 and 2 : tab
CTRL-C to quit
"""
obstacle = {'a', 's', 'd', 'f'}

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

currentBot = 1

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .5
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

locker_dir = 0  # Locking direction, 1 = CW, -1 = CCW
BLOCKER_OMEGA = 0.0525
locker_ang = 0
locker_state = False
blocker_init = False

def lockerAngleUpdate(angle):
    global locker_ang
    locker_ang = angle.process_value

def lockAction(key_pressed):
    global locker_dir, blocker_init, locker_state

    current_angle = float(round(locker_ang, 3) % PI)
    residual = 0

    if key_pressed == 't' :
        locker_dir = 0
        locker_state = True
        return locker_ang
     
    elif key_pressed == 'b' :
        locker_state = True
        if locker_dir == 0 :
            if not blocker_init:
                blocker_init = True
                locker_dir = 1
            else : 
                locker_dir = (current_angle - PI/2)/abs(current_angle - PI/2)
        else : locker_dir *= -1  
    
    if locker_state : 
        if locker_dir == 1: 
            residual = PI - current_angle
        elif locker_dir == -1:
            residual = current_angle - 0

        if residual > BLOCKER_OMEGA:
            return locker_ang + locker_dir*BLOCKER_OMEGA
        else : 
            locker_state = False
            return locker_ang
    else : 
        return locker_ang


def switch_bot(key_pressed):
    global current_bot 


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    #rospy.init_node('solamr_teleop'.format(robot_ns), anonymous=True)
    robot_ns = "solamr_1"
    vel_pub_1 = rospy.Publisher('/{0}/cmd_vel'.format(robot_ns), Twist, queue_size=5)
    vel_pub_2 = rospy.Publisher('/solamr_2/cmd_vel', Twist, queue_size=5)
    blocker_pub = rospy.Publisher('/{0}/blocker_position_controller/command'.format(robot_ns), Float64, queue_size=5)
    locker_sub = rospy.Subscriber("/{0}/blocker_position_controller/state".format(robot_ns), JointControllerState, lockerAngleUpdate )

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
#	    elif key in obstacle.keys():
	
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            vel_pub_1.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

            # BLOCKER SECTION

            target_angle = lockAction(key)
            blocker_pub.publish(target_angle)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        vel_pub_1.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
