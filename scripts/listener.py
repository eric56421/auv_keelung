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
    real = 0.25*value+7
    real2 = -0.25*value+7
    if command == 'right':
        pR.ChangeDutyCycle(8.2)
        pL.ChangeDutyCycle(8.2)
    
    elif command == 'left':        
        pR.ChangeDutyCycle(6.8)
        pL.ChangeDutyCycle(6.8)
    else:
        pR.ChangeDutyCycle(real)  
        pL.ChangeDutyCycle(real2)

def Command(data):
    global command

    if data.data == 'front':
        command = 'front'
    elif data.data == 'right':
        command = 'right'
    elif data.data == 'left':
        command = 'left'
    elif data.data == 'back':
        command = 'back'
    else:
        command = 'stable'
    
    
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
