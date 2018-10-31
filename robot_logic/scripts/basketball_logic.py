2#!/usr/bin/env python
# basketball_logic.py -- This node should do the main logic of the
# robot as described in the state diagram. It assumes we want to score in the
# blue basket now.

# Copyright (C) 2018 Marien Raat <marienraat@riseup.net>

# Author: Marien Raat <marienraat@riseup.net>

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import serial
import rospy
import math
from robot_hardware.comport_mainboard import ComportMainboard
from std_msgs.msg import String
import time
from ast import literal_eval

# Constants
CENTER_REGION = 0.05
CENTER_BASKET_REGION = 0.05
TIME_MOVING_FORWARD = 5
TIME_THROWING = 3
DISTANCE_TO_CENTER_BALL = 0.2
CAMERA_FOV = 29.0 # Trial and error (mostly error though)


class State:
    STOPPED, FORWARD, TURNING, MOVING_AND_TURNING, \
        CENTERING, THROWING = range(6)


def log(text):
    TAG = "robot_logic/basketball_logic"
    rospy.loginfo(TAG + ": " + text)

class BasketballLogic:
    def __init__(self):
        self.turn_left = True
        self.movement_publisher = rospy.Publisher("movement", String, queue_size=10)
        self.move_to_state(state.STOPPED)

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)

    def react(self, position, baskets):
        if self.state == State.STOPPED:
            self.react_stopped(position, baskets)
        elif self.state == State.FORWARD:
            self.react_forward(position, baskets)
        elif self.state == State.TURNING:
            self.react_turning(position, baskets)
        elif self.state == State.MOVING_AND_TURNING:
            self.react_moving_and_turning(position, baskets)
        elif self.state == State.CENTERING:
            self.react_centering(position, baskets)
        elif self.state == State.THROWING:
            self.react_throwing(position, baskets)

    def react_stopped(self, position, baskets):
        self.send("stop")

    def react_forward(self, position, baskets):
        if (time.time() - self.forward_start_time) > TIME_MOVING_FORWARD:
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        self.send("forward")

    def react_turning(self, position, baskets):
        if postion != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        if self.turn_left:
            self.send("turn_left")
        else:
            self.send("turn_right")

    def react_moving_and_turning(self, position, baskets):
        if position == "None":
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        pos = float(position.split(":")[0])
        distance = float(position.split(":")[1])

        if distance < DISTANCE_TO_CENTER_BALL:
            self.move_to_state(State.CENTERING)
            self.react(position, baskets)
            return

        rotational_speed = max(min(pos, 0.1), -0.1) * 100

        self.send("movement:30:{}:{}".format(pos * CAMERA_FOV,
                                             rotational_speed))

    def react_centering(self, position, baskets):
        if position == "None":
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        blue_basket_message = baskets.split(":")[0]
        log("Blue basket: {}".format(blue_basket_message))
        blue_basket = literal_eval(blue_basket_message)
        if blue_basket is None:
            log("Basket not found, assume it is on the left!")
            horizontal_basket_position = -1.0
        else:
            (bx, by, bw, bh) = blue_basket
            horizontal_basket_position = bx + (bw / 2.0) - 0.5
            log("Basket position is {}".format(horizontal_basket_position))

        if horizontal_basket_position >= -CENTER_BASKET_REGION and \
           horizontal_basket_position <= CENTER_BASKET_REGION:
            self.move_to_state(State.THROWING)
            self.react(position, baskets)
            return

        if horizontal_basket_position > CENTER_BASKET_REGION:
            self.send("movement:2:-90:8")
        elif horizontal_basket_position < -CENTER_BASKET_REGION:
            self.send("movement:2:90:-8")

    def react_throwing(self, position, baskets):
        if (time.time() - self.throwing_start_time) > TIME_THROWING:
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        self.throw()

    def move_to_state(self, state):
        if state == state.FORWARD:
            self.forward_start_time = time.time()
        elif state == state.THROWING:
            self.throwing_start_time = time.time()
        self.state = state

    def throw(self):
        self.send("forward")
        self.send("throw:200")

logic = BasketballLogic()

def new_object_callback(message):
    logic.react(message.data.split("\n")[0],
                  message.data.split("\n")[1])


def new_referee_command_callback(message):
    if message.data == "start":
        logic.move_to_state(State.FORWARD)
    elif message.data == "stop":
        logic.move_to_state(State.STOPPED)


if __name__ == "__main__":
    rospy.init_node("ball_thrower")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    rospy.Subscriber("robot_main/referee", String, new_referee_command_callback)

    rospy.spin()
