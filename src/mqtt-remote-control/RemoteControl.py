import tkinter as tk
from paho.mqtt import client as mqtt_client
import random


broker = 'broker.hivemq.com'
port = 1883
topic = "python/mqtt"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'

root = tk.Tk()
client = None

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

class Example(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent, width=400,  height=400)

        self.label = tk.Label(self, text="last key pressed:  ", width=20)
        self.label.pack(fill="both", padx=100, pady=100)

        self.label.bind("<w>", self.on_wasd)
        self.label.bind("<a>", self.on_wasd)
        self.label.bind("<s>", self.on_wasd)
        self.label.bind("<d>", self.on_wasd)
        self.label.bind("<space>", self.on_wasd)
        self.label.bind("<e>", self.on_wasd)
        self.label.bind("<x>", self.on_wasd)
        



        # give keyboard focus to the label by default, and whenever
        # the user clicks on it
        self.label.focus_set()
        self.label.bind("<1>", lambda event: self.label.focus_set())

    def on_wasd(self, event):
        global client
        publish(client, event.keysym)
        self.label.configure(text="last key pressed: " + event.keysym);


if __name__ == "__main__":
    client = connect_mqtt()
    client.loop_start()

    Example(root).pack(fill="both", expand=True)
    root.mainloop()
