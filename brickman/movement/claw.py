class Claw:
    """ Class to represent the claw on the robot """

    def __init__(self, motor):
        self.motor = motor
        self.holding = False

    def close(self):
        if self.holding:
            return
        self.motor.run_to_rel_pos(position_sp=-1800, speed_sp=1000,
                                  stop_action="brake")
        self.holding = True
        self.motor.wait_while('running')

    def open(self):
        if not self.holding:
            return
        self.motor.run_to_rel_pos(position_sp=1800, speed_sp=1000,
                                  stop_action="brake")
        self.holding = False
        self.motor.wait_while('running')
