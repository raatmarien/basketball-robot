#!/usr/bin/env python
# ball_centerer.py -- Attempts to center the biggest ball in the
# camera (horizontally)

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

movement_publisher = rospy.Publisher("movement", String, queue_size=10)

def new_object_callback(position):
    pos = float(position)
    rospy.loginfo("Ball at: " + position)
    if position > 0.1:
        movement_publisher.publish("turn_right")
    elif position < -0.1:
        movement_publisher.publish("turn_left")
    else:
        movement_publisher.publish("stop")

if __name__ == "__main__":
    rospy.init_node("ball_centerer")
    rospy.Subscriber("image_processing/object", String, new_object_callback)

    rospy.spin()
