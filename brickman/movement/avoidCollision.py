#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep

# Connect gyro sensor and check connected.
gy = GyroSensor()

assert gy.connected, "Connect a gyro sensor to any sensor port"

# Put the gyro sensor into GYRO-ANG mode
# to measure the turn angle in degrees
gy.mode='GYRO-RATE'
gy.mode='GYRO-ANG'

# Attach large motors to ports B and C
mB = LargeMotor('outB')
mC = LargeMotor('outC')


def turn():

    # Start the left motor with speed 40% to initiate a medium turn right.
    mB.run_forever(speed_sp=200)

    # Wait until the gyro sensor detects that the robot has turned
    # (at least) 45 deg in the positive direction (to the right)
    while gy.value() < 90 and gy.value()>-90:
        print("Gyro Value",gy.value())
        sleep(0.01)

    mB.stop()

    # Wait for the motion to finish
    mB.wait_while('running')



def main():
    # Connect ultrasonic and touch sensors to any sensor port
    # and check they are connected.
    us = UltrasonicSensor()
    assert us.connected, "Connect a single US sensor to any sensor port"
    ts = TouchSensor();    assert ts.connected, "Connect a touch sensor to any port"
    # can have 2 statements on same line if use semi colon

    # Put the US sensor into distance mode.
    us.mode='US-DIST-CM'

    units = us.units

    run_straight()

    # reports 'cm' even though the sensor measures 'mm'
    while not ts.value():    # Stop program by pressing touch sensor button
        # US sensor will measure distance to the closest
        # object in front of it.
        distance = us.value()/10  # convert mm to cm

        print(str(distance) + " " + units)


        if distance < 18:  #This is an inconveniently large distance
            Leds.set_color(Leds.LEFT, Leds.RED)
            mB.stop()
            mC.stop()
            sleep(0.04)
            turn()
        else:
            Leds.set_color(Leds.LEFT, Leds.GREEN)

    Sound.beep()
    Leds.set_color(Leds.LEFT, Leds.GREEN)  #set left led green before exiting

if __name__ == "__main__":
    main()
