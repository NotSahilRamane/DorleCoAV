
import random
import rospy
from paho.mqtt import client as mqtt_client
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import Twist     

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
throttle = 1 
brake = 0
AEB_FLAG = 0

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

def AEBCallback(data):
    global throttle, brake, AEB_FLAG
    throttle = data.linear.x
    brake = data.linear.y
    if brake > 0.5:
        AEB_FLAG = 1
    else:
        AEB_FLAG = 0

def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        control_command = CarlaEgoVehicleControl()
        global controlcommandPub
        global throttle, brake, AEB_FLAG
        m_decode=str(msg.payload.decode())
        
        m_separated = m_decode.split(";")
        if m_separated[0] == "CheckFlag":
            if AEB_FLAG == 1:
                control_command.throttle = throttle
                control_command.brake = brake
                # AEB_FLAG = 0
            else:
                control_command.throttle = float(m_separated[1])
                control_command.brake = float(m_separated[3])
                control_command.hand_brake = True if m_separated[5] == "True" else False
            if brake == 1:
                    control_command.hand_brake = True
                    control_command.brake = 1
            control_command.reverse = True if m_separated[2] == "True" else False
            control_command.steer = float(m_separated[4])
            control_command.manual_gear_shift = False
            controlcommandPub.publish(control_command)

    client.subscribe(topic)
    client.on_message = on_message

def run():
    rospy.init_node('MQTT_Controller')
    rospy.Subscriber("/vehicle/controls", Twist, AEBCallback)
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
