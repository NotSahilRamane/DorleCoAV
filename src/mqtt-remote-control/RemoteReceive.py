#!/usr/bin/python3

import random
import rospy
from paho.mqtt import client as mqtt_client
from carla_msgs.msg import CarlaEgoVehicleControl


# broker = 'broker.emqx.io'
# port = 1883
# topic = "python/mqtt"

broker = 'broker.hivemq.com'
port = 1883
topic = "python/mqtt"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'

throttle = 0
steering = 0
brake = 0
reverse = 0

controlcommandPub = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1)

def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        global throttle, steering, brake, reverse, controlcommandPub
        print(msg.payload.decode())
        control_command = CarlaEgoVehicleControl()
        if msg.payload.decode() == "w":
            throttle = 0.5
        if msg.payload.decode() == "a":
            steering = -0.9
        if msg.payload.decode() == "s":
            throttle = 0
        if msg.payload.decode() == "d":
            steering = 0.9
        if msg.payload.decode() == "x":
            steering = 0.0
        if msg.payload.decode() == "space":
            if brake == 0:
                brake = 1
                control_command.hand_brake = True
            elif brake == 1:
                brake = 0
                control_command.hand_brake = True
        if msg.payload.decode() == "e":
            if reverse == 0:
                reverse = 1
                control_command.reverse = True
            elif reverse == 1:
                reverse = 0
                control_command.reverse = False


        
        control_command.throttle = throttle
        control_command.steer = steering
        control_command.brake = brake
        control_command.manual_gear_shift = False
        controlcommandPub.publish(control_command)
        


    client.subscribe(topic)
    client.on_message = on_message


def run():
    rospy.init_node('MQTT_Controller')
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
