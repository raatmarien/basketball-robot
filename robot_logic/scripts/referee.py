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

#ser = serial.Serial('COM3',baudrate=115200,timeout = 1,dsrdtr=True)

fieldID = 'A'
robotID = 'B'


class Referee:
    def __init__(self):
        #self.turn_left = True
        self.referee_publisher = rospy.Publisher("referee", String, queue_size=10)
        #self.start_looking_time = time.time() + TIME_MOVING_FORWARD

	self.ser = serial.Serial(
	    'COM3',
	    baudrate=115200,
	    timeout = 1,
	    dsrdtr=True)

    def send(self, command):
        log("Sending {}".format(command))
        self.referee_publisher.publish(command)


    def listen(self):
        log("Listening for XBee")
    	answer = self.ser.read(12);
        log("Answer is {}".format(answer))
    	if answer == 'a' + fieldID + 'XSTART----' or answer == 'a' + fieldID + robotID + 'START----':
            self.ser.write( 'a' + fieldID + robotID + 'ACK-----')
            log("Got order to start")
            movement = True
    	elif answer == 'a' + fieldID + 'XSTOP-----' or answer == 'a' + fieldID + robotID + 'STOP-----':
            self.ser.write( 'a' + fieldID + robotID + 'ACK-----')
            log("Got order to stop")
            movement = False
	    self.send("stop")
        elif answer ==  'a' + fieldID + robotID + 'PING-----':
            log("Got order to ping")
            self.ser.write( 'a' + fieldID + robotID + 'ACK-----')


if __name__ == "__main__":
    rospy.init_node("referee")
    rate = rospy.Rate(10)

    try:
        referee = Referee()
    except:
	log("Failed to connect with Xbee")
        exit()

    while not rospy.is_shutdown():
        referee.listen()
        rate.sleep()

