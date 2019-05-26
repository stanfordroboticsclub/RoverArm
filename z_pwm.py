
from __future__ import division

import RPi.GPIO as GPIO
import time

from UDPComms import Subscriber, timeout
import math

target_vel = Subscriber(8410, timeout = 1)

pwm_pin = 14 #TODO CAHNGE
dir_pin = 4 #TODO CAHNGE

GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)

pwm = GPIO.PWM(pwm_pin, 50)
GPIO.output(dir_pin, True)

pan.start(0)
time.sleep(1)

while True:
    try:
        cmd = target_vel.get()['z']
    except timeout:
        print "TIMEOUT No commands received"
        print('driving z at', 0)
        pwm.ChangeDutyCycle(0)
    except:
        print('driving z at', 0)
        pwm.ChangeDutyCycle(0)
        raise
    else:
        if cmd < 0:
            GPIO.output(dir_pin, False)
            print('down')
        else:
            GPIO.output(dir_pin, True)
            print('up')

        # pwm_duty is in % so 0 - 100
        # but motor is 12V and we are using 40V
        # so don't got above 25%
        pwm_duty = 14*math.fabs(cmd)
        print('Driving z at',pwm_duty)
        pwm.ChangeDutyCycle(pwm_duty)
    finally:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
        GPIO.output(pwm_pin, False)
        GPIO.cleanup()
    # exit()



