#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys
import rclpy
import time

from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from lift_control.action import Command
from std_msgs.msg import String as StringMsg

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
space key, s : force stop

Lift control:
q : Lift up
e : Lift down
r : Stop lift (emergency stop - stops movement immediately)
t : Cancel lift action (cancels current goal)

CTRL-C to quit

Lift State: {}
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


class LiftActionClient:
    """Action client for controlling the lift mechanism."""
    
    def __init__(self, node):
        self.node = node
        self._action_client = ActionClient(node, Command, '/lift_action')
        self.goal_handle = None
        self.current_lift_status = "Unknown"
        self.count = 0
        # Create subscriber for lift state
        self._state_subscriber = node.create_subscription(
            StringMsg,
            '/lift_state',
            self._state_callback,
            10
        )
        
        print("Waiting for lift action server...")
        self._action_client.wait_for_server()
        print("Lift action server connected!")
    
    def _state_callback(self, msg):
        """Callback for lift state updates."""
        self.current_lift_status = msg.data
        print(f'Lift State: {self.current_lift_status}')
        self.count += 1

    
    def send_command(self, command):
        """Send lift command (0=STOP, 1=UP, 2=DOWN)."""
        if self.goal_handle is not None:
            print('Previous lift goal still active, canceling it first...')
            self.cancel_goal()
        
        goal_msg = Command.Goal()
        goal_msg.command = command
        
        cmd_str = ["STOP", "GO UP", "GO DOWN"]
        print(f'Sending lift command: {cmd_str[command]}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def cancel_goal(self):
        """Cancel the current goal."""
        if self.goal_handle is None:
            print('No active lift goal to cancel')
            return
        
        print('Canceling lift goal...')
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)
    
    def _cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            print('Lift goal successfully canceled')
        else:
            print('Lift goal cancellation failed')
        self.goal_handle = None
    
    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Lift command REJECTED')
            self.goal_handle = None
            return
        
        print('Lift command ACCEPTED')
        self.goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _feedback_callback(self, feedback_msg):
        """Handle feedback during goal execution."""
        feedback = feedback_msg.feedback
        status_str = "MOVING" if feedback.status == 0 else "REACHED"
        print(f'Lift feedback: {status_str}')
        print(f'Lift State: {self.current_lift_status}')
    
    def _result_callback(self, future):
        """Handle final result."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            completion_str = "Reached" if result.completion == 1 else "Stopped"
            print(f'Lift result: {completion_str}')
        elif status == 5:  # CANCELED
            print('Lift result: Canceled')
        elif status == 6:  # ABORTED
            print('Lift result: Aborted')
        print(f'Lift State: {self.current_lift_status}')
        self.goal_handle = None


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    # Create lift action client
    lift_action_client = LiftActionClient(node)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg.format(lift_action_client.current_lift_status))
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                lift_action_client.count += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                lift_action_client.count += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                lift_action_client.count += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                lift_action_client.count += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'q':
                # Lift up
                lift_action_client.send_command(1)
            elif key == 'e':
                # Lift down
                lift_action_client.send_command(2)
            elif key == 'r':
                # Stop lift (send STOP command - stops movement at current position)
                print('Emergency stop - stopping lift at current position')
                lift_action_client.send_command(0)
            elif key == 't':
                # Cancel current lift action
                lift_action_client.cancel_goal()
            else:
                if (key == '\x03'):
                    break

            if lift_action_client.count == 10:
                print(msg.format(lift_action_client.current_lift_status))
                lift_action_client.count = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)
            
            # Spin once to process action callbacks
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
