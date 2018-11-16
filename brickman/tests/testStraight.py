#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import time
from claw import Claw
from robot import Robot
import os.path

def main():

    if not(os.path.exists("/home/robot/testStraight.rtf")):
        print('file does not exist')
        xfile = open('testStraight.rtf','w')
        header = '{0: <25}'.format('Power Level') + '{0: <30}'.format('Distance Covered') + '{0: <20}'.format('Time needed') + '{0: <25}'.format('Orientation After') + '{0: <25}'.format('Speed') + '\n'
        xfile.write(header)
        xfile.close()

    xfile = open('testStraight.rtf','a')
    powerSupply = PowerSupply()

    powers = []
    for i in range(0,10):
        powers.append(powerSupply.measured_volts)
    power = sum(powers)/ float(len(powers))

    # Connect gyro sensor and check connected.
    gy = GyroSensor()
    assert gy.connected, "Connect a gyro sensor to any sensor port"

    # Connect front ultrasonic and left side sensors to any sensor port
    usFront = UltrasonicSensor('in4')
    assert usFront.connected, "Connect a single ultrasonic sensor to  port 4 in front of the robot"
    usSide = UltrasonicSensor('in1')
    assert usSide.connected, "Connect a a single ultrasonic sensor to  port 1 in the left side of the robot"

    cs = ColorSensor()
    assert cs.connected, "Connect a color sensor to any port"
    # Attach large motors to ports B and C
    leftMotor = LargeMotor('outC')
    rightMotor = LargeMotor('outB')
    clawMotor = MediumMotor('outA')
    robot = Robot(leftMotor, rightMotor, clawMotor, gy, cs, usFront, usSide)
    robot.findCorner()

    speeds = {200,400,600,700,800,900}
    for speed in speeds:
        sleep(5)
        robot.resetGyro()
        initialDistance = robot.getDistanceFront()

        # item found is for knowing if runStraight finish after an item is found so the robot knows that it has to start from the beggining
        initialTime = time.time()

        wallFound = not robot.runStraight(speed)
        if wallFound == True:
            endTime = time.time() - initialTime
            orientation = robot.getGyroValue()
            new_entry = '{0: <25}'.format(str(round(power,5))+'volts') +'{0: <30}'.format(str(round(abs(initialDistance-robot.getDistanceFront()),5)) + 'cm')+  '{0: <20}'.format(str(round(endTime,5)) + 'sec')  + '{0: <25}'.format(str(orientation) + 'degrees') + '{0: <25}'.format(str(speed) + 'm/s')+'\n'

            xfile.write(new_entry)

            Sound.beep()
    xfile.close()


if __name__ == "__main__":
    main()
