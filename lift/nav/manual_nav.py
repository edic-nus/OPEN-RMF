import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from paho.mqtt import client as mqtt_client
import random

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


class ManualNav(Node):
    def __init__(self):
        super().__init__('manual_nav')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.get_logger().info("Use W/S to increase/decrease forward speed, A/D to increase/decrease angular speed, 'up' to make the lift go up, and 'down' to make the lift go down. Press q to quit.")

    def mover(self, client):
        twist = Twist()
        command = ''
        while command != 'q':
            command = str(input())
            if command == 'w':
                self.linear_speed += 0.01
            elif command == 'x':
                self.linear_speed -= 0.01
            elif command == 'a':
                self.angular_speed += 0.1
            elif command == 'd':
                self.angular_speed -= 0.1
            elif command == 's':
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            elif command == 'up':
                client.publish(topic, "up")
            elif command == 'down':
                client.publish(topic, "down")
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.publisher_.publish(twist)
            self.get_logger().info(f"Publishing -> Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist)
        self.get_logger().info(f"Quitting... Stopping the robot")


def main(args=None):
    rclpy.init(args=args)
    manual_nav = ManualNav()
    client = connect_mqtt()

    client.loop_start()
    manual_nav.mover(client)
    
    manual_nav.destroy_node()
    rclpy.shutdown()
    client.loop_stop()


if __name__ == '__main__':
    main()
