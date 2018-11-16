#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
from time import sleep

# MQTT Publisher
send_msg = {
    'command': 'turn',
    'payload':{
      'degrees': 15,
    }
}

client = mqtt.Client()
client.connect("localhost",1883,60)
flag = True

client.publish("json", payload=json.dumps(send_msg))
print(json.dumps(send_msg))
sleep(1)

send_msg = {
    'command': 'run_straight',
    'payload':{
      'speed': 500,
    }
}

print(json.dumps(send_msg))
client.publish("json", payload=json.dumps(send_msg))
sleep(1)
print('3rd')

send_msg = {
    'command': 'run_straight',
    'payload':{
      'speed': 50,
    }
}
print(json.dumps(send_msg))
client.publish("json", payload=json.dumps(send_msg))

# client.publish("topic/test", "Q");
client.disconnect();