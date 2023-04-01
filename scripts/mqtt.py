#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import rospy
from paho.mqtt import client as mqtt_client
from std_msgs.msg import String

broker = 'broker.emqx.io'
port = 1883
topic = "NCT/mqtt/channel/positions"

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client):
    def on_message(client, userdata, msg):
        message = msg.payload.decode()
        # print("Received `{1}` from `{2}` topic".format(msg.payload.decode(), msg.topic))
        mes = String().data = message
        pub.publish(mes)
        print(message)
        
    client.subscribe(topic)
    client.on_message = on_message

def main():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

if __name__ == '__main__':
    rospy.init_node('mqtt_server', anonymous=True)
    pub = rospy.Publisher('mqtt_server', String, queue_size=10)
    main()