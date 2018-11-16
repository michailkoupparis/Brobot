#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json

# MQTT Subscriber

# m = MediumMotor(OUTPUT_A)
client = mqtt.Client()
client.connect("localhost",1883,60)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe([("topic/test",2), ("json",2)])

def on_message(client, userdata, msg):
    if msg.topic == 'json':
      print(json.loads(msg.payload.decode("utf-8")))
      send_msg = json.loads(msg.payload.decode("utf-8"))
      client.publish("json", payload=json.dumps(send_msg))
    if msg.payload.decode("utf-8") == 'Q':
      client.disconnect()

# client.connect("192.168.105.117",1883,60)

client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()