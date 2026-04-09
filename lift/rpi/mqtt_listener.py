import random
from paho.mqtt import client as mqtt_client
import RPi.GPIO as GPIO
from time import sleep
import threading

broker = 'broker.emqx.io'
port = 1883
topic = "lift"
client_id = f'subscribe-{random.randint(0, 100)}'

# GPIO Pins
DIR = 10
STEP = 8
CW = 1
CCW = 0
HALL_PIN_1 = 11
HALL_PIN_2 = 13

GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(HALL_PIN_1, GPIO.IN)
GPIO.setup(HALL_PIN_2, GPIO.IN)

# Stepper params
MAX_SPEED_DELAY = 0.0075
MIN_SPEED_DELAY = 0.06
ACCELERATE_RAMP_STEPS = 50
DECELERATE_RAMP_STEPS = 30

# 🔥 GLOBAL STATE
STOP = False
CURRENT_THREAD = None
LOCK = threading.Lock()


def ramped_step(direction, stop_condition):
    global STOP

    GPIO.output(DIR, direction)
    step_count = 0

    while not stop_condition():
        if STOP:
            print("🛑 FORCE STOP triggered!")
            break

        # Acceleration
        if step_count < ACCELERATE_RAMP_STEPS:
            delay = MIN_SPEED_DELAY - (MIN_SPEED_DELAY - MAX_SPEED_DELAY) * (step_count / ACCELERATE_RAMP_STEPS)
        else:
            delay = MAX_SPEED_DELAY

        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

        step_count += 1

    # Deceleration
    print("Decelerating...")
    for i in range(DECELERATE_RAMP_STEPS):
        delay = MAX_SPEED_DELAY + (MIN_SPEED_DELAY - MAX_SPEED_DELAY) * (i / DECELERATE_RAMP_STEPS)
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)

    STOP = False
    print("Motion stopped.")


def go_up():
    print("Going up to Level 1")
    ramped_step(CW, lambda: GPIO.input(HALL_PIN_1) == GPIO.LOW)
    print("Done UP")


def go_down():
    print("Going down to Ground level")
    ramped_step(CCW, lambda: GPIO.input(HALL_PIN_2) == GPIO.LOW)
    print("Done DOWN")


# 🔥 THREAD LAUNCHER
def start_motion(target_func):
    global CURRENT_THREAD, STOP

    with LOCK:
        # Stop existing motion
        STOP = True
        if CURRENT_THREAD and CURRENT_THREAD.is_alive():
            CURRENT_THREAD.join()

        # Reset stop flag
        STOP = False

        # Start new motion thread
        CURRENT_THREAD = threading.Thread(target=target_func)
        CURRENT_THREAD.start()


def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print(f"Failed to connect, return code {rc}")

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client):
    def on_message(client, userdata, msg):
        global STOP

        command = msg.payload.decode()
        print(f"Received `{command}`")

        if command == "up":
            start_motion(go_up)

        elif command == "down":
            start_motion(go_down)

        elif command == "stop":
            print("🛑 STOP command received")
            STOP = True

    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
