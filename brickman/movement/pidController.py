#!/usr/bin/env python3
from time import sleep

from ev3dev.auto import *

# ------Input--------
power = 60
target = 45
kp = float(0.7) # Proportional gain, Start value 1
kd = 0.7           # Derivative gain, Start value 0
ki = float(0.02) # Integral gain, Start value 0
minAngle = 0
maxAngle = 90
direction = -1
# -------------------

# Connect two large motors on output ports B and C and check that
# the device is connected using the 'connected' property.
left_motor = LargeMotor(OUTPUT_C);  assert left_motor.connected
right_motor = LargeMotor(OUTPUT_B); assert right_motor.connected
# One left and one right motor should be connected

# Connect color and touch sensors and check that they
# are connected.
# ir = InfraredSensor();        assert ir.connected
ts = TouchSensor();     assert ts.connected
gy = GyroSensor();      assert gy.connected
# col= ColorSensor();   assert col.connected
gy.mode='GYRO-ANG'
gy.mode='GYRO-RATE'

# Change color sensor mode
# col.mode = 'COL-REFLECT'

# Adding button so it would be possible to break the loop using
# one of the buttons on the brick
btn = Button()

def steering(course, power):
		"""
		Computes how fast each motor in a pair should turn to achieve the
		specified steering.
		Input:
				course [-100, 100]:
				* -100 means turn left as fast as possible,
				*  0   means drive in a straight line, and
				*  100  means turn right as fast as possible.
				* If >100 pr = -power
				* If <100 pl = power
		power: the power that should be applied to the outmost motor (the one
				rotating faster). The power of the other motor will be computed
				automatically.
		Output:
				a tuple of power values for a pair of motors.
		Example:
				for (motor, power) in zip((left_motor, right_motor), steering(50, 90)):
						motor.run_forever(speed_sp=power)
		"""

		power_left = power_right = power
		s = (50-abs(float(course))) / 50

		if course >= 0:
				power_right *= s
				if course > 100:
						power_right = - power
		else:
				power_left *= s
				if course < -100:
						power_left = - power

		return (int(power_left), int(power_right))

def run(power, target, kp, kd, ki, direction, minAngle, maxAngle):
		"""
		PID controlled line follower algoritm used to calculate left and right motor power.
		Input:
				power. Max motor power on any of the motors
				target. Normalized target value.
				kp. Proportional gain
				ki. Integral gain
				kd. Derivative gain
				direction. 1 or -1 depending on the direction the robot should steer
				minRef. Min reflecting value of floor or line
				maxRef. Max reflecting value of floor or line
		"""
		lastError = error = integral = 0
		left_motor.run_direct()
		right_motor.run_direct()
		while not btn.any() :
				if ts.value(): # User pressed the touch sensor
						print('Breaking loop')
						break
				refRead = gy.value() + 45
				print("Gyro ",gy.value())
				error = target - (100 * ( refRead - minAngle ) / ( maxAngle - minAngle ))
				derivative = error - lastError
				lastError = error
				integral = float(0.5) * integral + error
				course = (kp * error + kd * derivative +ki * integral) * direction
				#print(course)
				for (motor, pow) in zip((left_motor, right_motor), steering(course, power)):
						motor.duty_cycle_sp = pow
				sleep(0.01) # Aprox 100 Hz

run(power, target, kp, kd, ki, direction, minAngle, maxAngle)

# Stop the motors before exiting.
print('Stopping motors')
left_motor.stop()
right_motor.stop()
