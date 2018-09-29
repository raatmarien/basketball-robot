#!/usr/bin/env python
# ball_centered_driver.py -- Attempts to drive to the biggest ball in
# view

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

# Constants
CENTER_REGION = 0.1

driver = BallCenteredDriver()

def log(text):
    TAG = "robot_logic/ball_centered_driver"
    rospy.loginfo(TAG + ": " + text)

class BallCenteredDriver:
    def __init__(self):
        self.turn_left = True
        self.movement_publisher = rospy.Publisher("movement", String, queue_size=10)

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)

    def react(self, position):
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
                if distance < 1.0:
                    self.send("stop")
                else:
                    self.send("forward")
        else:
            log("We don't see a ball, so we turn till we do")
            if self.turn_left:
                self.send("turn_left")
            else:
                self.send("turn_right")
                


def new_object_callback(message):
    driver.react(message.data.split("\n")[0])


if __name__ == "__main__":
    rospy.init_node("ball_centered_driver")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    
    rospy.spin()
