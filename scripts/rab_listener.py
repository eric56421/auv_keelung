#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

pR=GPIO.PWM(32,50)# 50hz frequency
pL=GPIO.PWM(12,50)

pR.start(7)
pL.start(7)

pR.ChangeDutyCycle(7)
pL.ChangeDutyCycle(7)

command = 'front'

def control(value):
    global command
    
    value = value.data
    real = -1.1*value+7
    print(real)
     
    if command == 'back':
        pR.ChangeDutyCycle(7.8)
        pL.ChangeDutyCycle(7)
    elif command == 'front':        
        pR.ChangeDutyCycle(7)
        pL.ChangeDutyCycle(7.8)
    else:
        pR.ChangeDutyCycle(real)  
        pL.ChangeDutyCycle(real)
    
def Command(data):
    global command
    command = data.data
    print(command)
        
def listener():    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('command', String, Command)
    rospy.Subscriber('value' , Float32 , control)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print("STOP")
    finally:
        GPIO.cleanup()
    
