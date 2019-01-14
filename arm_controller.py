

from UDPComms import Subscriber
from roboclaw_interface import RoboClaw
import math


FIRST_LINK = 1000
SECOND_LINK = 1000

# native angles = 0 at extension

def find_serial_port():
    return '/dev/ttyUSB0'
    return '/dev/ttyACM0'
    return '/dev/tty.usbmodem1141'

class Vector(list):

    def __add__(self,other):
        assert len(self) == len(other)
        return Vector( a+b for a,b in zip(self,other) )

    def __sub__(self,other):
        assert len(self) == len(other)
        return Vector( a-b for a,b in zip(self,other) )

    def __mul__(self,other):
        return Vector( other * a for a in self )

    def __rmul__(self,other):
        return Vector.__mul__(self,other)  

    def norm(self):
        return math.sqrt(sum(a**2 for a in self))

    def __repr__(self):
        return "Vector(" + super().__repr__() + ")"

class Arm:
    def __init__(self):
        self.target = Subscriber(8410)

        self.xyz_names = ["x", "y"]

        self.motor_names = ["shoulder",
                            "elbow"]

        self.native_positions = { motor:0 for motor in self.motor_names}
        self.CPR = { motor:100 for motor in self.motor_names}

        self.rc = RoboClaw(find_serial_port(), names = self.motor_names, addresses = [128] ) # addresses = [128, 129, 130])


    def get_location(self):
        for motor in self.motor_names:
            self.native_positions[motor] = self.rc.get_encoder(motor)/self.CPR[motor]

        self.xyz_positions = self.native_to_xyz(self.native_positions)

    def xyz_to_native(self, xyz):
        pass

    def native_to_xyz(self, native):
        pass
        # self.native_positions['x'] = 
        # self.native_positions['y'] = 

    def update(self):
        current_location = Vector(self.get_location())
        target_location = Vector( self.target.get() )

        delta = (target_location - current_location) 

        delta * (1/delta.norm())


