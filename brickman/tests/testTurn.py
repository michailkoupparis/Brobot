#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import time
from claw import Claw
from robot import Robot
import os.path


def turn(robot, degrees,speed):


   if(degrees < 0):
        robot.turnDirection = -1
   else:
       robot.turnDirection = 1

   if robot.turnDirection == -1:
       mTurn = robot.rightMotor
   else:
       mTurn = robot.leftMotor

   mTurn.run_forever(speed_sp=speed)

   #Wait until the gyro sensor detects that the robot has turned
   #while abs(gyro.value()) < degrees-2 and time.time() < endtime:
   #    sleep(0.2)
   while abs(robot.getGyroValue()) < abs(degrees):
       print('Gyro Value',robot.getGyroValue())
       print('Deegrees',degrees)
       sleep(0.01)

   mTurn.stop(stop_action='brake')
   mTurn.run_forever(speed_sp=0)
   
   sleep(0.3)
   gyroValue = abs(robot.getGyroValue())
   print('Gyro Value After', gyroValue)
   

def main():

    if not(os.path.exists("/home/robot/testTurn.rtf")):
        print('file does not exist')
        xfile = open('testTurn.rtf','w')
        header = '{0: <25}'.format('Power Level') + '{0: <20}'.format('Time needed') + '{0: <20}'.format('Degrees') + '{0: <25}'.format('Actual Turn') +'{0: <20}'.format('Speed') + '\n'
        xfile.write(header)
        xfile.close()

    xfile = open('testTurn.rtf','a')
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

    degrees = {45,90,-90,-60}
    speeds = {150,200,250}
    for degree in degrees:
        for speed in speeds:
            robot.resetGyro()
            initialTime = time.time()
            print(degree)
            print(speed)
            turn(robot,degree,speed)
            endTime = time.time() - initialTime
            new_entry = '{0: <25}'.format(str(round(power,5))+'volts') + '{0: <20}'.format(str(round(endTime,5)) + 'sec') + '{0: <20}'.format(str(degree) + 'degrees') + '{0: <25}'.format(str(round(robot.getGyroValue(),3))+'degrees') + '{0: <20}'.format(str(speed) + 'm/s') + '\n'
            xfile.write(new_entry)
            robot.resetGyro()
            Sound.beep()
            
    xfile.close()
if __name__ == "__main__":
    main()
