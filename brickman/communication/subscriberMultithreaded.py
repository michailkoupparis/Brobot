#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
from ev3dev.auto import *
from robot import Robot
from time import sleep
from runStraightThread import RunStraightThread

# MQTT Subscriber

# m = MediumMotor(OUTPUT_A)

gy = GyroSensor()
assert gy.connected, "Connect a gyro sensor to any sensor port"

# Connect front ultrasonic and left side sensors to any sensor port
usFront = UltrasonicSensor('in4')
assert usFront.connected, "Connect a single ultrasonic sensor to  port 4 in front of the robot"

cs = ColorSensor()
assert cs.connected, "Connect a color sensor to any port"
# Attach large motors to ports B and C
leftMotor = LargeMotor('outC')
rightMotor = LargeMotor('outB')
clawMotor = MediumMotor('outA')
distanceFromWall = 25
robot = Robot(leftMotor, rightMotor, clawMotor, gy, cs, usFront)
global threadRunning, goStraightThread
goStraightThread = RunStraightThread(robot= robot, speed=700)
threadRunning = False
robot.resetGyro()


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    Sound.beep()
    client.subscribe([("topic/test",2), ("json",2)])

def on_disconnect(client, userdata, rc):
    print("Disconnected")

def on_message(client, userdata, msg):
    global threadRunning, goStraightThread
    print("MESSAGE RECEIVED")
    if msg.topic == 'json':
      print(json.loads(msg.payload.decode("utf-8")))
      send_msg = json.loads(msg.payload.decode("utf-8"))
      command = send_msg['command']

      if command == 'turn':
          if threadRunning:
              goStraightThread.stop()
              threadRunning = False
              sleep(0.1)
          degrees = int(send_msg['payload']['degrees'])
          robot.resetGyro()
          sleep(0.1)
          robot.turn(degrees)
          client.publish("finished", "finished")
     
      elif command == 'stop':
          if threadRunning:
              goStraightThread.stop()
              threadRunning = False
              sleep(0.1)

          if robot.claw.holding:
              robot.claw.open()
          client.disconnect()
          

      elif command == 'returned_home':
          if robot.itemFound:
              robot.claw.open()
              robot.itemFound = False
              robot.runForward(1000,300,True)
          client.publish("finished", "finished")
      
      elif command == 'close_claw':
          if threadRunning:
              goStraightThread.stop()
              threadRunning = False
              sleep(0.1)
          robot.claw.close()
      
      elif command == 'run_straight':
          if threadRunning:
              goStraightThread.stop()
              sleep(0.1)

          speed = int(send_msg['payload']['speed'])
          goStraightThread = RunStraightThread(robot= robot, speed=speed)
          goStraightThread.start()
          threadRunning = True

      # Message to finished topic means robot finished executing the function
     
      

    if msg.payload.decode("utf-8") == 'Q':
      # m.stop()
      client.disconnect()

HOST = "192.168.105.117"
client = mqtt.Client(clean_session=True)
client.connect("192.168.105.117",1883,120)

client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

client.loop_forever()
