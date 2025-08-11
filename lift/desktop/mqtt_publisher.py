import random
import time
import keyboard  # Requires `pip install keyboard`

from paho.mqtt import client as mqtt_client

broker = 'broker.emqx.io'
port = 1883
topic = "lift"
client_id = f'publish-{random.randint(0, 1000)}'

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client):
    print("Press 'w' to send 'up', 's' to send 'down', or 'q' to quit.")
    while True:
        if keyboard.is_pressed('w'):
            client.publish(topic, "up")
            print("Sent 'up'")
            time.sleep(0.3)  # debounce delay
        elif keyboard.is_pressed('s'):
            client.publish(topic, "down")
            print("Sent 'down'")
            time.sleep(0.3)
        elif keyboard.is_pressed('q'):
            print("Exiting...")
            break

def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    client.loop_stop()

if __name__ == '__main__':
    run()
