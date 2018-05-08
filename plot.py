#!/usr/bin/env python
import time
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


phi_f, x_f, y_f = 0,0,0
x_l, y_l = 0,0
v,w = 0,0
v_f,w_f = 0,0
x_actual = [] ; y_actual = []
x_desired = [] ; y_desired = []
v_input = []; w_input = []
v_input_f = []; w_input_f = []
key_inp =0
def callback_follower(msg):
    global phi_f, x_f, y_f
#    print "at callback"
    x_f = msg.linear.x
    y_f = msg.linear.y
    phi_f = msg.angular.x

def callback_leader(msg):
    global x_l, y_l
#    print "at callback"
    x_l = msg.linear.x
    y_l = msg.linear.y

def callback_key(message):
    global key_inp
    print "at callback"
    key_inp = message.data
    print "key_inp:", key_inp

def callback_velocity_l(msg):
    global v, w
#    print "at callback"
    v = msg.linear.x
    w = msg.linear.y

def callback_velocity_f(msg):
    global v_f, w_f
#    print "at callback"
    v_f = msg.linear.x
    w_f = msg.linear.y

def listener():
        rospy.init_node('plotter',anonymous=True)
        rospy.Subscriber('F_states', Twist, callback_follower)
        rospy.Subscriber('Odometry', Twist, callback_leader)
        rospy.Subscriber("keys", Int32, callback_key)
        rospy.Subscriber('Leader_commands', Twist, callback_velocity_l)
        rospy.Subscriber('Follower_commands', Twist, callback_velocity_f)


if __name__=='__main__':
    try:
        global phi_f, x_f, y_f, x_l, y_l, key_inp, x_desired, y_desired, x_actual, y_actual, key_inp, v, w, v_input, w_input, v_input_f, w_input_f
        listener()
        while not rospy.is_shutdown():
            x_actual.append(x_f)
            y_actual.append(y_f)
            x_desired.append(x_l)
            y_desired.append(y_l)
            v_input.append(v)
            w_input.append(w)
            v_input_f.append(v_f)
            w_input_f.append(w_f)

            time.sleep(.5)
            if key_inp == 5:
                length = [t for t in range(1, len(x_desired)+1)]
                length = np.array(length)
                x_d = np.array(x_desired)
                y_d = np.array(y_desired)
                x_a = np.array(x_actual)
                y_a = np.array(y_actual)
                v_i = np.array(v_input)
                w_i = np.array(w_input)
                v_i_f = np.array(v_input_f)
                w_i_f = np.array(w_input_f)
                plt.figure(1)
                plt.plot(length, x_a, 'x', length, x_d,'o')
                plt.xlabel('time')
                plt.ylabel('x')
                plt.figure(2)
                plt.plot(length, y_a, 'x', length, y_d,'o')
                plt.xlabel('time')
                plt.ylabel('y')
                plt.figure(3)
                plt.subplot(222)
                plt.plot(length, w_i_f, 'o')
                plt.xlabel('time')
                plt.ylabel('w_f')
                plt.subplot(221)
                plt.plot(length, v_i_f, 'x')
                plt.xlabel('time')
                plt.ylabel('velocity_f')
                plt.subplot(223)
                plt.plot(length, v_i, 'x')
                plt.xlabel('time')
                plt.ylabel('velocity_l')
                plt.subplot(224)
                plt.plot(length, w_i, 'o')
                plt.xlabel('time')
                plt.ylabel('w_l')

                plt.show()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
