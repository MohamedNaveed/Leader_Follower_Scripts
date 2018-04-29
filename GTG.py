#!/usr/bin/env python
from geometry_msgs.msg import Twist
import rospy
import sys,tty,termios
import serial
import time
import math

ser = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(2) # wait for Arduino

x_goal = 0; y_goal = 0
wheel_radius = 2.55 #cm
wheel_distance = 15 #cm
prev_encoder_l=0; prev_encoder_r=0
x, y, phi = 0,0,0
sum_error_phi = 0.0; sum_error_v = 0.0;
MIN_VEL = 70 #50
MAX_WH_VEL = 20 # 160


def callback_bot(msg):
    global x_goal, y_goal
#    print "at callback"
    x_goal = msg.linear.x
    y_goal = msg.linear.y
#    print msg.linear.x, msg.linear.y

def listener():
    rospy.init_node('follower_bot',anonymous=True)
    rospy.Subscriber('Odometry', Twist, callback_bot)


def odom(d_l,d_r):
    global phi, x, y

    d_theta_l = (d_l*6.28/30)
    d_theta_r = (d_r*6.28/30)
    ds = wheel_radius*(d_theta_l + d_theta_r)/2
    d_phi = wheel_radius*(d_theta_l - d_theta_r)/wheel_distance
    phi = phi + d_phi
    d_x = ds*math.cos(phi)
    d_y = ds*math.sin(phi)
    x = x + d_x
    y = y + d_y
    print "x:",x," y:", y, "phi:", phi

def control(x_g, y_g, x, y):
    global sum_error_phi, sum_error_v
    Kp_v = 0.1;    Ki_v = 0.0004 #0.001
    Kp_w = 0.45;   Ki_w = 0  #0.003
    if abs(y_g - y) < 2.0 and abs(x_g-x) < 2.0:
        return 0,0
    else:
        dist = math.sqrt(math.pow(y_g-y, 2) + math.pow(x_g-x, 2))
    #    if y_g > y and x_g > x:
        phi_d = math.atan2((y_g - y), (x_g - x))
    #    else:
    #        phi_d = math.atan2((y_g - y), (x_g - x)) + 3.14
    #        dist = -dist
        #print "phi_goal:", phi_d

        #print "dist:", dist
        sum_error_v = sum_error_v + dist
        v = Kp_v*dist + Ki_v*sum_error_v
        error_phi = math.atan2(math.sin(phi_d - phi), math.cos(phi_d - phi))
        sum_error_phi = sum_error_phi + error_phi
        w = Kp_w*(error_phi) + Ki_w*(sum_error_phi)
        #print "v:" ,v, " w:", w
        v_r = (2*v - w*wheel_distance)/2*wheel_radius
        v_l = (2*v + w*wheel_distance)/2*wheel_radius
        print "b4 scaling v_r:", v_r, "b4 scaling v_l:", v_l
        if v_r > 0.01:
            v_r = (v_r/MAX_WH_VEL)*(255 - MIN_VEL) + MIN_VEL
        elif v_r < -0.01:
            v_r = (v_r/MAX_WH_VEL)*(255 - MIN_VEL) - MIN_VEL
        if v_l > 0.01:
            v_l = (v_l/MAX_WH_VEL)*(255 - MIN_VEL) + MIN_VEL
        elif v_l < -0.01:
            v_l = (v_l/MAX_WH_VEL)*(255 - MIN_VEL) - MIN_VEL

        print "after scaling v_r:", v_r, "b4 scaling v_l:", v_l
        max_val = max(abs(v_r), abs(v_l))

        if max_val > 255:
            if v_r > 0.01:
                v_r = (v_r/max_val)*(255 - MIN_VEL) + MIN_VEL
            elif v_r < -0.01:
                v_r = (v_r/max_val)*(255 - MIN_VEL) - MIN_VEL
            if v_l > 0.01:
                v_l = (v_l/max_val)*(255 - MIN_VEL) + MIN_VEL
            elif v_l < -0.01:
                v_l = (v_l/max_val)*(255 - MIN_VEL) - MIN_VEL


        return v_l, v_r

if __name__ == "__main__":
    listener()
    while not rospy.is_shutdown():
        print "x_goal:", x_goal, "y_goal", y_goal

        vel_w_l, vel_w_r = control(x_goal, y_goal, x, y)
        msg = str(int((vel_w_l)))+":"+str(int((vel_w_r)))+":"
        print "vel_l:", vel_w_l, "vel_r:", vel_w_r
        ser.write(msg)
        time.sleep(0.2)
        message =  ser.read(ser.inWaiting())
        mes = message.split("\n")
        #print "mes:", mes
        #print "mes1", mes[1]
        #print "L:", mes[1].split(":")[0]
        #print "R:", mes[1].split(":")[1]

        encoder_l = float(mes[1].split(":")[0])
        encoder_r = float(mes[1].split(":")[1])
        odom(encoder_l-prev_encoder_l , encoder_r-prev_encoder_r)
        prev_encoder_l = encoder_l
        prev_encoder_r = encoder_r


    rospy.spin()

#         while True:
#
#
#             #if ser.is_open :
#             #message =  ser.read(ser.inWaiting())
#             #message =  ser.read(20)
#             #print "message:", message
#             #             #ser.flush()
#     except rospy.ROSInterruptException:
#         pass
