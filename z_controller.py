
from __future__ import division

from UDPComms import Subscriber, timeout
from roboclaw_interface import RoboClaw
import math
import time

# native angles = 0 at extension
# native angles = positive in the math direction

def find_serial_port():
    return '/dev/serial0'

class Arm:
    def __init__(self):
        self.target_vel = Subscriber(8410)

        self.pwm_names = ["z", "z_clone"]
        self.rc = RoboClaw(find_serial_port(), names=self.pwm_names, addresses = [131])


        try:
            while 1:
                start_time = time.time()
                self.update()
                while (time.time() - start_time) < 0.1:
                    pass

        except KeyboardInterrupt:
            print('driving z at', 0)
            self.rc.drive_duty('z', 0)
            raise
        except:
            print('driving z at', 0)
            self.rc.drive_duty('z', 0)
            raise


    def update(self):
        print()
        print("new iteration")
 
        try:
            target = self.target_vel.get()
            # TODO: This shouldn't be necessary, how to fix in UDPComms?
            target = {bytes(key): value for key, value in target.iteritems()}

#            target_f = self.condition_input(target)

        except timeout:
            print "TIMEOUT No commands recived"
            print('driving z at', 0)
            self.rc.drive_duty('z', 0)
        except:
            print('driving z at', 0)
            self.rc.drive_duty('z', 0)
            raise
        else:
            print('Driving z at',int(19000*target['z']))
            self.rc.drive_duty('z',int(-19000*target['z']))
        # exit()




if __name__ == "__main__":
    a = Arm()
