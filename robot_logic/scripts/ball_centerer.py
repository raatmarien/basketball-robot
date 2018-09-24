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

turn_left = True

def new_object_callback(position):
    global turn_left
    if position.data == "None":
        # WAY TO HACKY
        if turn_left:
            pos = -1.5
        else:
            pos = 1.5
        distance = -1
    else:            
        pos = float(position.data.split(":")[0])
        distance = float(position.data.split(":")[0])
    rospy.loginfo("Logic: Ball at: " + str(pos))
    if pos > 0.1:
        turn_left = False
        rospy.loginfo("Sending turn_right")
        movement_publisher.publish("turn_right")
    elif pos < -0.1:
        turn_left = True
        rospy.loginfo("Sending turn_left")
        movement_publisher.publish("turn_left")
    elif distance > 1.0:
        rospy.loginfo("Sending forward")
        movement_publisher.publish("forward")
    else:
        movement_publisher.publish("stop")

if __name__ == "__main__":
    rospy.init_node("ball_centerer")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)

    rospy.loginfo("lets start turning left")
    movement_publisher.publish("turn_left")
    
    rospy.spin()
