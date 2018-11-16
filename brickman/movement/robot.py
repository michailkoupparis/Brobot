#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
from claw import Claw
import time


class Robot:
  """ Class for generic API and internal state of the robot"""

  def __init__(self, leftMotor, rightMotor, clawMotor, gyro, colorSensor, ultraSonicFront):
    self.leftMotor = leftMotor
    self.rightMotor = rightMotor
    self.claw = Claw(clawMotor)
    # Sensors
    self.ultraSonicFront = ultraSonicFront
    self.ultraSonicFront.mode = 'US-DIST-CM'
    self.colorSensor = colorSensor
    self.colorSensor.mode='COL-COLOR'
    self.gyro = gyro
    # Set default turn direction
    # Turn direction: left: -1, right: 1
    self.turnDirection = -1
    # Set default starting corner
    # Corner: -1 left corner , 1 right corner
    self.startingCorner = 1
    # No item found initially
    self.itemFound = False
    self.forceStop = False



  def getDistanceSide(self):
    # Side distance in cm
    return self.ultraSonicSide.value()/10

  def getDistanceFront(self):
    # Front distance in cm
    return self.ultraSonicFront.value()/10

  def getGyroValue(self):
    return self.gyro.value()

  def getColorSensorValue(self):
    if self.claw.holding:
      return 0

    return self.colorSensor.value()

  def findCorner(self):
    """ Sets starting starting corner """
    sleep(0.1)
    # For some reason getDistance is not working
    #if getDistanceSide() < 20:
    if self.ultraSonicSide.value()/10 < 20:
        self.startingCorner = -1
    else:
        self.startingCorner = 1

    self.turnDirection = -1 * self.startingCorner


  def resetGyro(self):
      # Resets the gyro and calibrates it.
      # Seems like a hacky way to do it but that's how it is done in official docs
      self.gyro.mode = 'GYRO-RATE'
      self.gyro.mode = 'GYRO-ANG'


  def runForward(self, time, speed, backwards = False):
    # No clue why it doesnt work with longer time interval...
    if(backwards):
        speed *= -1
    for i in range(0, time//50):
        if self.getColorSensorValue() != 0:
            return True

        self.rightMotor.run_timed(time_sp=50, speed_sp=speed)
        self.leftMotor.run_timed(time_sp=50, speed_sp=speed)
    return False

  def runStraight(self,speed):
    self.resetGyro()
    # TODO: Export these constants to a seperate file
    Kp = 20
    offset = 0
    Tp = speed
    powerB = 700
    powerC = 700
    self.rightMotor.run_forever(speed_sp=Tp)
    self.leftMotor.run_forever(speed_sp=Tp)
    endtime = time.time()
    print('Time to start the straight',endtime)
    while not self.forceStop and time.time()<endtime+0.50:
    # Make the robot advance for 100 milliseconds
    # (50% speed, apply brake when movement terminated)
       print('Time in the straight',endtime)
       gyroValue = self.getGyroValue()
       error = gyroValue - offset
       Turn = (Kp * error)
       if Turn >= 0:
           powerB = min(1000, Tp + Turn)
           powerC = max(-1000,Tp - Turn)
       else:
           powerB = max(-1000, Tp + Turn)
           powerC = min(1000,  Tp - Turn)

       self.rightMotor.run_forever(speed_sp=powerB)
       self.leftMotor.run_forever(speed_sp=powerC)

       if self.colorSensor.value() != 0 and not self.claw.holding:
           self.stopRobot()
           self.itemFound = True
           return True

       if self.getDistanceFront() < 25:
           self.stopRobot()
           return False

    self.stopRobot()
    return False

  def turn(self, degrees,speed = 100):

    print('Deegrees to Turn')
    if(degrees < 0):
        self.turnDirection = -1
    else:
        self.turnDirection = 1

    if self.turnDirection == -1:
        print('Turn Left')
        mTurn = self.rightMotor
    else:
        print('Turn Right')
        mTurn = self.leftMotor

    mTurn.run_forever(speed_sp=speed)
    print(self.getGyroValue())
    print(degrees)

    #Wait until the gyro sensor detects that the robot has turned
    #while abs(gyro.value()) < degrees-2 and time.time() < endtime:
    #    sleep(0.2)
    while abs(self.getGyroValue()) < abs(degrees):
        sleep(0.01)

    mTurn.stop(stop_action='brake')
    mTurn.run_forever(speed_sp=0)


    sleep(0.3)
    gyroValue = abs(self.getGyroValue())
    #If it passess the deegrees or fall short turn to reach the expected value
    #  if deegrees are not between 0 and 3 , 2 for the error of the gyroscope and 1 to have an accceptanc einterval
    # If the robot turned more turn the other direction the error angle to make the initial turn better
    if not(abs(gyroValue - degrees) >=0 and abs(gyroValue -degrees) <= 2):
        if abs(gyroValue)>abs(degrees):
            self.resetGyro()
            #turn the opposite direction
            if(degrees < 0):
                sign = 1
            else:
                sign = -1
            print(sign*(abs(gyroValue)-abs(degrees)))
            self.turn(sign*(abs(gyroValue)-abs(degrees)),speed)
            return
        # If the robot turned less turn the same direction the error angle to make the initial turn better
        elif abs(gyroValue)<abs(degrees):
            self.resetGyro()
            if(degrees < 0):
                sign = -1
            else:
                sign = 1
            print(sign*(abs(degrees)-abs(gyroValue)))
            self.turn(sign*(abs(degrees) -abs(gyroValue)),speed)
            return

    self.resetGyro()


  def stopRobot(self):
      print('Strop Robot')
      self.forceStop = True
      #stop all the motors
      self.rightMotor.stop(stop_action="hold")
      self.leftMotor.stop(stop_action="hold")
      self.rightMotor.run_forever(speed_sp=0)
      self.leftMotor.run_forever(speed_sp=0)
      sleep(1)
