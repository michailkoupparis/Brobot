#!/usr/bin/env python3
from ev3dev.ev3 import *
from robot import Robot
from time import sleep
from runStraightThread import RunStraightThread


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
distanceFromWall = 25

robot = Robot(leftMotor, rightMotor, clawMotor, gy, cs, usFront, usSide)
robot.turn(15)
sleep(1)
thread = RunStraightThread(robot= robot, speed=700)
print("STARTING RUNSTRAIGHT THREAD")
thread.start()
sleep(2)
print("STOPPING RUNSTRAIGHT THREAD")
thread.stop()
thread = RunStraightThread(robot=robot, speed=100)
thread.start()
sleep(3)
thread.stop()
sleep(0.1)
robot.turn(15)
sleep(1)
