#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep

# Connect gyro sensor and check connected.
gy = GyroSensor()

assert gy.connected, "Connect a gyro sensor to any sensor port"

# Put the gyro sensor into GYRO-ANG mode
# to measure the turn angle in degrees
gy.mode='GYRO-ANG'

# Attach large motors to ports B and C
mB = LargeMotor('outB')
mC = LargeMotor('outC')


gy_before = gy.value()

# Start the left motor with speed 40% to initiate a medium turn right.
mB.run_forever(speed_sp=180)
mC.run_forever(speed_sp=-180)

# Wait until the gyro sensor detects that the robot has turned
# (at least) 45 deg in the positive direction (to the right)
while (gy_before - gy.value()) < 88: # loop until turn angle exceeds 45 degrees
    print("Gyro Value",gy_before-gy.value())
    sleep(0.01)

mB.stop()
mC.stop()
# Wait for the motion to finish
mB.wait_while('running')
mC.wait_while('running')