try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
import rospy
from paho.mqtt import client as mqtt_client
import json
import random

broker = 'broker.hivemq.com'
port = 1883
topic = "python/mqtt"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'

client = None

pygame.init()
pygame.display.set_mode((500, 500))
running = True


def connect_mqtt():
    global username, password
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

def publish(client, data):
    msg = str(data)
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")


class KeyboardControl(object):
    """
    Handle input events
    """

    def __init__(self,):

        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

    
        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=1)

    # pylint: disable=too-many-branches
    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1

        
        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        global publish, client
        self._control.throttle = 0.5 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = bool(keys[K_SPACE])
        data_out = json.dumps(self._control)
        publish(client, data_out)



        
client = connect_mqtt()
client.loop_start()
clock = pygame.time.Clock()

getConrol = KeyboardControl()

while running:
    KeyboardControl.parse_events(clock)
