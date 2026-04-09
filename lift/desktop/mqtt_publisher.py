import random
from paho.mqtt import client as mqtt_client

# MQTT Configuration
broker = 'broker.emqx.io'
port = 1883
topic = "lift"
client_id = f'publish-{random.randint(0, 1000)}'


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ Connected to MQTT Broker!")
        else:
            print(f"❌ Failed to connect, return code {rc}")

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    print("\n🎮 Lift Control Panel")
    print("---------------------")
    print("w → Move UP")
    print("s → Move DOWN")
    print("x → 🛑 FORCE STOP")
    print("q → Quit\n")

    while True:
        key = input("Enter command: ").strip().lower()

        if key == 'w':
            client.publish(topic, "up")
            print("⬆️ Sent 'up'")
        elif key == 's':
            client.publish(topic, "down")
            print("⬇️ Sent 'down'")
        elif key == 'x':
            client.publish(topic, "stop")
            print("🛑 Sent 'STOP'")
        elif key == 'q':
            print("👋 Exiting publisher...")
            break
        else:
            print("⚠️ Invalid input. Use w/s/x/q.")


def run():
    client = connect_mqtt()
    client.loop_start()   # Run MQTT network loop in background
    publish(client)
    client.loop_stop()


if __name__ == '__main__':
    run()
