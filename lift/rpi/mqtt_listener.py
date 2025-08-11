import random
from paho.mqtt import client as mqtt_client
import RPi.GPIO as GPIO
from time import sleep, time

broker = 'broker.emqx.io'
port = 1883
topic = "lift"
# Generate a Client ID with the subscribe prefix.
client_id = f'subscribe-{random.randint(0, 100)}'

# Direction pin from controller
DIR = 10
# Step pin from controller
STEP = 8
# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0
# Hall effect sensors
HALL_PIN_1 = 11  # Level 1
HALL_PIN_2 = 13  # Ground

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(HALL_PIN_1, GPIO.IN)
GPIO.setup(HALL_PIN_2, GPIO.IN)

# Stepper motor parameters
MAX_SPEED_DELAY = 0.0075  # Fastest step rate (lower is faster)
MIN_SPEED_DELAY = 0.06   # Slowest step rate (startup/slowdown)
ACCELERATE_RAMP_STEPS = 50         # Number of steps to accelerate/decelerate
DECELERATE_RAMP_STEPS = 30

def ramped_step(direction, stop_condition):
    GPIO.output(DIR, direction)
    
    step_count = 0
    while not stop_condition():
        # Apply a proportional-like ramping effect
        if step_count < ACCELERATE_RAMP_STEPS:
            delay = MIN_SPEED_DELAY - (MIN_SPEED_DELAY - MAX_SPEED_DELAY) * (step_count / ACCELERATE_RAMP_STEPS)
        else:
            delay = MAX_SPEED_DELAY

        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

        step_count += 1
    
    print("Decelerating")
    for decelerate_count in range(0, DECELERATE_RAMP_STEPS):
        delay = MAX_SPEED_DELAY - (MAX_SPEED_DELAY - MIN_SPEED_DELAY) * (decelerate_count / DECELERATE_RAMP_STEPS)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

def go_up():
    print("Going up to Level 1")
    ramped_step(CW, lambda: GPIO.input(HALL_PIN_1) == GPIO.LOW)
    print("Reached Level 1")

def go_down():
    print("Going down to Ground level")
    ramped_step(CCW, lambda: GPIO.input(HALL_PIN_2) == GPIO.LOW)
    print("Reached Ground Level")


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
        command = msg.payload.decode()
        topic = msg.topic
        print(f"Received `{command}` from `{topic}` topic")
        if command == "up":
            go_up()
        elif command == "down":
            go_down()

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
