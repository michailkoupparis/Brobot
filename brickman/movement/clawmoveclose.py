#!/usr/bin/env python3
from ev3dev.ev3 import *
from claw import Claw

from time import sleep

# Attach large motors to ports B and C
cs = ColorSensor()
ts = TouchSensor()
claw = Claw(MediumMotor('outA'))
cs.mode='COL-COLOR'
# Make the robot advance such that the wheels rotate 720 deg
# (50% speed, apply brake when movement terminated).
# Assuming speed_sp=900 gives full speed then
# speed_sp=450 gives 50% speed
print(cs.value())
holding = False
print(sys.path)
while not ts.value():
        print("Running...")
        while cs.value() != 0 and not holding:
                Sound.speak('I see red').wait()
                claw.close()
                # wait for both motors to complete their movements
                mA.wait_while('running')
                sleep(1) # Wait one second
                holding = True

        while ts.value():
                Sound.speak('I will open now').wait()
                claw.open()
                mA.wait_while('running')
                sleep(1)
                holding = False
