
import time
from roboclaw_interface import RoboClaw


rc = RoboClaw('/dev/serial0', names=[0,1,2,3], addresses=[128,129])

for i in range(4):
    rc.set_encoder(i,0)


while 1:
    print [ rc.read_encoder(i) for i in range(4)]

