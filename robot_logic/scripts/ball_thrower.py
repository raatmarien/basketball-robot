#!/usr/bin/env python
# ball_thrower.py -- This module has the goal of throwing one ball in
# the direction of the basket. This ball will be placed in the center
# of the field. We assume we will throw it in the blue basket. For
# this we follow the protocol of ball_and_basket_centerer and then we
# will throw.

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
TIME_MOVING_FORWARD = 10


def log(text):
    TAG = "robot_logic/ball_thrower"
    rospy.loginfo(TAG + ": " + text)

class BallThrower:
    def __init__(self):
        self.turn_left = True
        self.movement_publisher = rospy.Publisher("movement", String, queue_size=10)
        self.start_looking_time = time.time() + TIME_MOVING_FORWARD
        self.throwing = False

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)

    def throw(self):
        self.send("forward")
        self.send("throw:2000")
        
    def react(self, position, baskets):
        log(position)

        if self.throwing:
            log("Throwing")
            self.throw()
            return
        
        # Do we see a ball?
        if position != "None":
            pos = float(position.split(":")[0])
            distance = float(position.split(":")[1])
            # Is it in the center?
            if pos > CENTER_REGION:
                self.turn_left = False
                self.send("turn_right")
            elif pos < -CENTER_REGION:
                self.turn_left = True
                self.send("turn_left")
            else:
                # Is it close?
                if distance < 0.25:
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
                    # Is the basket centered?
                    if horizontal_basket_position > CENTER_BASKET_REGION:
                        self.send("movement:2:-90:8")
                    elif horizontal_basket_position < -CENTER_BASKET_REGION:
                        self.send("movement:2:90:-8")
                    else:
                        # Throw here
                        self.throw()
                        self.throwing = True
                else:
                    self.send("forward")
        else:
            if time.time() < self.start_looking_time:
                log("We don't see the ball, so first we will move forward")
                self.send("forward")
            else:
                log("We don't see a ball, so we turn till we do")
                if self.turn_left:
                    self.send("turn_left")
                else:
                    self.send("turn_right")
                

thrower = BallThrower()

def new_object_callback(message):
    thrower.react(message.data.split("\n")[0],
                  message.data.split("\n")[1])


if __name__ == "__main__":
    rospy.init_node("ball_thrower")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    
    rospy.spin()
