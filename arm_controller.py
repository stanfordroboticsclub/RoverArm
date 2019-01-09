

from UDPComms import Subscriber
from roboclaw_interface import RoboClaw
import math


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



    def get_location(self):
        pass

    def xyz_to_native(self, xyz):
        pass

    def native_to_xyz(self, native):
        pass

    def update(self):
        current_location = Vector(self.get_location())
        target_location = Vector( self.target.get() )

        delta = (target_location - current_location) 

        delta * (1/delta.norm())


