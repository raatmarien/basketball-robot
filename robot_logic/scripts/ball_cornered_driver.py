#!/usr/bin/env python
# ball_cornered_driver.py -- Attempts to drive to the biggest ball in
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
CAMERA_FOV = 40 # See https://digilabor.ut.ee/index.php/Cube-O-Bot#Mechanics

driver = BallCorneredDriver()

def log(text):
    TAG = "robot_logic/ball_cornered_driver"
    rospy.loginfo(TAG + ": " + text)

class BallCorneredDriver:
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
            # Is it on the edge?
            if pos < 0.4:
                self.turn_left = False
                self.send("turn_right")
            elif pos > 0.5:
                self.turn_left = True
                self.send("turn_left")
            else:
                # Is it close?
                if distance < 1.0:
                    self.send("stop")
                else:
                    self.send("movement:10:{}:0".format(CAMERA_FOV))
        else:
            log("We don't see a ball, so we turn right till we do")
            # Only turn right, since we try to put the ball at the
            # right edge of the view, so it probably went out that
            # way.
            self.send("turn_right")
                


def new_object_callback(message):
    driver.react(message.data.split("\n")[0])


if __name__ == "__main__":
    rospy.init_node("ball_cornered")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    
    rospy.spin()
