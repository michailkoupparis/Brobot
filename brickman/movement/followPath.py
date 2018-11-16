#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import time
from claw import Claw
from robot import Robot

# COMMENT
def goHome(robot):
    # Turn around so no longer facing the wall
    robot.turn(180,200)
    robot.runStraight(700)
    if(robot.turnDirection == -robot.startingCorner):
        robot.runStraight(700)

# afterForward is for knowing if it was called by the runForward or th runStraight function
def bringBackItem(robot, afterForward = False):
    # Take the object
    robot.claw.close()
    sleep(0.5)
    # If it find the object when it was in the runStraight
    if (afterForward == False):
        if(robot.turnDirection == -robot.startingCorner):
            robot.turn(180,200)

        robot.runStraight(700)
        robot.turnDirection *= -robot.startingCorner
        robot.turn(90,200)
        robot.runStraight(700)

    #  If it find the object when it was in the runForward
    else:
        if (robot.turnDirection == robot.startingCorner):
            robot.turn(180,200)
            robot.runStraight(700)
        else:
            robot.turn(90,200)
            robot.runStraight(700)
            robot.turn(90,200)
            robot.runStraight(700)

    robot.claw.open()
    sleep(0.5)
    robot.runForward(2000, 600, True)


def main():
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
    distanceFromWall = 25

    robot = Robot(leftMotor, rightMotor, clawMotor, gy, cs, usFront, usSide)
    robot.findCorner()

    while True:
        # item found is for knowing if runStraight finish after an item is found so the robot knows that it has to start from the beggining
        itemFound = robot.runStraight(700)
        if itemFound == True:
            bringBackItem(robot, False)
            robot.turn(90,200)
            # Change the itam Found to scan for new items
            robot.itemFound = False

        distance = robot.getDistanceFront()

        if distance < distanceFromWall:
            # Stop motors
            rightMotor.stop(stop_action='hold')
            leftMotor.stop(stop_action='hold')
            leftMotor.wait_while('running')
            rightMotor.wait_while('running')

            robot.turn(90,200)

            # If we are facing another wall we are in a corner - go to start
            if(robot.getDistanceFront() < distanceFromWall):
                goHome(robot)
                break
        # item found is for knowing if runForward finish after an item is found so the robot knows that it has to start from the beggining
            itemFound = robot.runForward(2000, 600)

            if itemFound == True:
                bringBackItem(robot, True)
                # Set the first turning direction and turn to face the reach the exact same position as the start
                robot.turnDirection = -robot.startingCorner
                robot.turn(90,200)
                # Change the itam Found to scan for new items
                robot.itemFound = False
            else:
                # If we are facing another wall we are in a corner - go to start
                if(robot.getDistanceFront() < distanceFromWall):
                    robot.runForward(2000, 600, True)
                    robot.leftMotor.wait_while('running')
                    robot.rightMotor.wait_while('running')
                    goHome(robot)
                    break
                # If nothing is found turn again an dcontinue to the next lane facing in the opposite direction
                robot.turn(90,200)
                robot.turnDirection *= -1

    Sound.beep()


if __name__ == "__main__":
    main()
