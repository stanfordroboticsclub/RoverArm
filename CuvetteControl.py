
from __future__ import division

from UDPComms import Subscriber, timeout
from roboclaw_interface import RoboClaw
import math
import time


def find_serial_port():
    return '/dev/serial0'

class Arm:
    def __init__(self):
        self.target_vel = Subscriber(8410)

        self.motor_names = ["spinner"]
        self.pwm_names = ["null"]

        self.CPR = {'spinner':1294}
        self.oneShot = 0
        self.rc = RoboClaw(find_serial_port(), names = self.motor_names + self.pwm_names,\
                                                    addresses = [128])
	self.pos = 0
       
        while 1:
            start_time = time.time()
            self.update()
            while (time.time() - start_time) < 0.1:
                pass


    def condition_input(self,target):
        target['x']     = - target['x']
        target['yaw']  = 0.01* target['yaw']

        # rotates command frame to end effector orientation
        angle = self.xyz_positions['yaw']
        x = target['x']
        y = target['y']
        target['x'] = x*math.cos(angle) - y*math.sin(angle)
        target['y'] = x*math.sin(angle) + y*math.cos(angle)

        return target

    def update(self):
        try:
            print()
            print("new iteration")
            target = self.target_vel.get()
            # TODO: This shouldn't be necessary, how to fix in UDPComms?
            target = {bytes(key): value for key, value in target.iteritems()}
            print(target)
            target_f = target
            if(target_f['grip'] == 1 and self.oneShot == 0):
                self.pos = self.pos + 1
                self.oneShot = 1
            elif(target_f['grip'] == -1 and self.oneShot == 0):
                self.pos = self.pos - 1
                self.oneShot = 1
            if(target_f['grip'] == 0):
                self.oneShot = 0
	
            if target_f["reset"]:
                print ("RESETTING!!!")
                self.rc.set_encoder('spinner',0)

            self.rc.drive_position('spinner',int(self.pos*self.CPR['spinner']))
        except timeout:
            print ("No commands") 


if __name__ == "__main__":
    a = Arm()
