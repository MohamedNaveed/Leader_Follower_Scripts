import math
import XBee
import socket
import serial
from time import sleep

def send(message):
  # Your serial port name here
      xbee = XBee.XBee("/dev/ttyACM2",115200)
#print "b4 send"
      xbee.SendStr(message,0x0000)
      sleep(0.25)


if __name__=="__main__":
    while(1):
        vel_w_1 = 100
        vel_w_2 = 100
        message = str(int((vel_w_1 - (vel_w_1>255)*(vel_w_1%255))+500))+":"+str(int((vel_w_2 - (vel_w_2>255)*(vel_w_2%255))+500))+":"
        #send(message)
        #print message
        xbee = XBee.XBee("/dev/ttyACM0",115200)
        recv_message = xbee.Receive()
        print recv_message
