#!/usr/bin/env python

import serial
import time
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2) # wait for Arduino
#vel_w_1 = 150
#vel_w_2 = 200
vel_w_l = 0 ; vel_w_r = 0
wheel_radius = 2.55 #cm
wheel_distance = 15 #cm
prev_encoder_l=0; prev_encoder_r=0
x, y, phi = 0,0,0
sum_error_phi = 0.0; sum_error_v = 0.0;
MIN_VEL = 70 #50
MAX_WH_VEL = 180 # 160
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

def callback_key(message):
    global key_inp
    print "at callback"
    key_inp = message.data
    print "key_inp:", key_inp
    get(key_inp)

def listener():
        rospy.init_node('tele_operation',anonymous=True)
        rospy.Subscriber("keys", Int32, callback_key)



def publish_odom():
    global x, y, phi
    msg1 = Twist()
    pub = rospy.Publisher("Odometry", Twist, queue_size=10)
    rate= rospy.Rate(10)
    print "x:", x

    msg1.linear.x = float(x)
    msg1.linear.y = float(y)
    msg1.angular.x = float(phi)
    #while not rospy.is_shutdown():
    pub.publish(msg1)

def get(k):
    print "give input"
    global vel_w_l, vel_w_r

    if k==1:
            print "up"
            vel_w_l, vel_w_r = 150, 150
            print "inside"
    elif k==2:
            print "down"
            vel_w_l, vel_w_r = 0, 0

    elif k==3:
            print "right"
            vel_w_l, vel_w_r = 150, 0

    elif k==4:
            print "left"
            vel_w_l, vel_w_r = 0, 150

    else:
            print "not an arrow key!"

if __name__=='__main__':
    try:
        global vel_w_l, vel_w_r
        listener()
        while not rospy.is_shutdown():

            print "vel_w_l:",vel_w_l

            print "vel_left:", vel_w_l, " vel_right:", vel_w_r
            msg = str(int((vel_w_l)))+":"+str(int((vel_w_r)))+":"
            ser.write(msg)
            time.sleep(0.2)
            print "sent"

            message =  ser.read(ser.inWaiting())
            #message =  ser.read(20)
            #print "message:", message
            mes = message.split("\n")
            #print "mes:", mes
            #print "mes1", mes[1]
            print "L:", mes[1].split(":")[0]
            print "R:", mes[1].split(":")[1]

            encoder_l = float(mes[1].split(":")[0])
            encoder_r = float(mes[1].split(":")[1])
            odom(encoder_l-prev_encoder_l , encoder_r-prev_encoder_r)
            prev_encoder_l = encoder_l
            prev_encoder_r = encoder_r
            publish_odom()
            #ser.flush()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
