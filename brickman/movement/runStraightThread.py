from stoppableThread import StoppableThread

class RunStraightThread(StoppableThread):
    def __init__(self, *args, **kwargs):
      super(RunStraightThread, self).__init__(target=self.run)
      self.robot = kwargs['robot']
      self.speed = kwargs['speed']

    def run(self):
      while not self.stopped():
        self.robot.resetGyro()
        # TODO: Export these constants to a seperate file
        Kp = 20
        offset = 0
        Tp = self.speed
        self.robot.rightMotor.run_forever(speed_sp=Tp)
        self.robot.leftMotor.run_forever(speed_sp=Tp)
        while not self.stopped():
        # Make the robot advance for 100 milliseconds
        # (50% speed, apply brake when movement terminated)

          gyroValue = self.robot.getGyroValue()
          error = gyroValue - offset
          Turn = (Kp * error)
          if Turn >= 0:
              powerB = min(1000, Tp + Turn)
              powerC = max(-1000,Tp - Turn)
          else:
              powerB = max(-1000, Tp + Turn)
              powerC = min(1000,  Tp - Turn)

          self.robot.rightMotor.run_forever(speed_sp=powerB)
          self.robot.leftMotor.run_forever(speed_sp=powerC)
          print("Claw ->" , self.robot.colorSensor.value())
        #   if int(self.robot.colorSensor.value()) != 0 and not self.robot.claw.holding:
        #       self.robot.itemFound = True
        #       self.robot.claw.close()
        #       break

        self.robot.rightMotor.stop()
        self.robot.leftMotor.stop()
        return False