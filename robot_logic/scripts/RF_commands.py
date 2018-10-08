#!/usr/bin/env python
# view

# Copyright (C) 2018 Magnus Kaldjaerv <kaldjarv.magnusgmail.com> and Marien Raat <marienraat@riseup.net>

# Authors: Magnus Kaldjaerv <kaldjarv.magnusgmail.com> and Marien Raat <marienraat@riseup.net>

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

def log(text):
    TAG = "robot_logic/ball_and_basket_centerer"
    rospy.loginfo(TAG + ": " + text)

ser = serial.Serial('COM3',baudrate=115200,timeout = 1,dsrdtr=True)

fieldID = 'A'
robotID = 'B'


class RFcommands:
    def __init__(self):
        #self.turn_left = True
        self.movement_publisher = rospy.Publisher("movement", String, queue_size=10)
        #self.start_looking_time = time.time() + TIME_MOVING_FORWARD

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)


while True:
    answer = ser.read(12);
    if answer == 'a' + fieldID + 'XSTART----' or answer == 'a' + fieldID + robotID + 'START----':
        ser.write( 'a' + fieldID + robotID + 'ACK-----')
        log("Got order to start")
        liikumine = True
    elif answer == 'a' + fieldID + 'XSTOP-----' or answer == 'a' + fieldID + robotID + 'STOP-----':
        ser.write( 'a' + fieldID + robotID + 'ACK-----')
        log("Got order to stop")
        liikumine = False
    elif answer ==  'a' + fieldID + robotID + 'PING-----':
        log("Got order to ping")
        ser.write( 'a' + fieldID + robotID + 'ACK-----')


if __name__ == "__main__":
    rospy.init_node("RF_commands")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)

    rospy.spin()