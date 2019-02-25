

from roboclaw_interface import RoboClaw

rc = RoboClaw('/dev/serial0', names=range(8), addresses=[128,129,130,131])

for i in range(8):
    print(rc.read_main_battery_voltage(i))


