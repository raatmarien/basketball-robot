#!/usr/bin/env python
# drive.py --- Sending movement commands to the main board

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
from std_msgs.msg import String

# Constants
# Wheel angles in degrees
WHEEL_ONE_ANGLE = 0
WHEEL_TWO_ANGLE = 120
WHEEL_THREE_ANGLE = 240

# Basic setting for communication with our main board. The dsrdtr
# setting enables hardware flow control.
main_board = serial.Serial("COM3", baudrate=115200, timeout=0.8, dsrdtr=True)


def get_relative_speed_for_wheel(wheel_angle, drive_angle):
    return math.cos(math.radians(drive_angle - wheel_angle))


def move_in_direction(degrees):
    speed = 20

    wheel_one_speed = speed * get_relative_speed_for_wheel(WHEEL_ONE_ANGLE, degrees)
    wheel_two_speed = speed * get_relative_speed_for_wheel(WHEEL_TWO_ANGLE, degrees)
    wheel_three_speed = speed * get_relative_speed_for_wheel(WHEEL_THREE_ANGLE, degrees)

    serial_message = ("sd:" + str(wheel_one_speed) + ":" + str(wheel_two_speed)
                      + ":" + str(wheel_three_speed) + "\r\n")

    main_board.write("f0\r\n".encode("utf-8"))
    main_board.write(serial_message.encode("utf-8"))


def movement_callback(command):
    if command == "forward":
        move_in_direction(0)
    elif command == "backward":
        move_in_direction(180)
    elif command == "left":
        move_in_direction(270)
    elif command == "right":
        move_in_direction(90)


def movement_listener():
    rospy.init_node("movement_listener")
    rospy.Subscriber("movement", String, movement_callback)

    rospy.spin()


if __name__ == "__main__":
    movement_listener()
