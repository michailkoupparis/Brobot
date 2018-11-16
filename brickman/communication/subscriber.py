#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
from ev3dev.auto import *
from robot import Robot
from time import sleep
# MQTT Subscriber

# m = MediumMotor(OUTPUT_A)

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

robot.resetGyro()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    Sound.beep()
    client.subscribe([("topic/test",2), ("json",2)])

def on_disconnect(client, userdata, rc):
    print("Disconnected")

def on_message(client, userdata, msg):
    print("MESSAGE RECEIVED")
    if msg.topic == 'json':
      print(json.loads(msg.payload.decode("utf-8")))
      send_msg = json.loads(msg.payload.decode("utf-8"))
      command = send_msg['command']

      if command == 'turn':
          degrees = int(send_msg['payload']['degrees'])
          robot.resetGyro()
          robot.turn(degrees)
      
      elif command == 'run_straight':
          print('Inside run Straight instruction')
          robot.stopRobot()
          sleep(1)
          robot.forceStop = False
          speed = int(send_msg['payload']['speed'])
          print('Speed is',speed)
          foundItem = robot.runStraight(speed)
          if foundItem:
              robot.claw.close()
      
      elif command == 'run_forward' :
          time = int(send_msg['payload']['time'])
          speed = int(send_msg['payload']['speed'])
          robot.runForward(time, speed)
      
      Sound.beep()
      Sound.beep()
      Sound.beep()
      # Message to finished topic means robot finished executing the function
      client.publish("finished", "finished")
      

    if msg.payload.decode("utf-8") == 'Q':
      # m.stop()
      client.disconnect()

HOST = "192.168.105.117"
client = mqtt.Client()
client.connect("192.168.105.117",1883,120)

client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

client.loop_forever()
