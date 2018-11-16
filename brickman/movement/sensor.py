#!/usr/bin/env python3
from ev3dev.ev3 import *


cs = ColorSensor()
cs.mode= 'COL-COLOR'

while True:
    print(cs.value())
