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
from robot_hardware.comport_mainboard import ComportMainboard
from std_msgs.msg import String

# Constants
# Wheel angles in degrees
WHEEL_ONE_ANGLE = 0
WHEEL_TWO_ANGLE = 120
WHEEL_THREE_ANGLE = 240

main_board = ComportMainboard()
main_board.run()

def get_relative_speed_for_wheel(wheel_angle, drive_angle):
    return math.cos(math.radians(drive_angle - wheel_angle))


def move_in_direction(degrees):
    speed = 20

    wheel_one_speed = speed * get_relative_speed_for_wheel(WHEEL_ONE_ANGLE, degrees)
    wheel_two_speed = speed * get_relative_speed_for_wheel(WHEEL_TWO_ANGLE, degrees)
    wheel_three_speed = speed * get_relative_speed_for_wheel(WHEEL_THREE_ANGLE, degrees)

    main_board.launch_motor(wheel_one_speed, wheel_two_speed, wheel_three_speed)


def movement_callback(command):
    command = str(command)[7:][:-1]
    rospy.loginfo("Recieved command: " + command)
    if command == "forward":
        move_in_direction(0)
    elif command == "backward":
        move_in_direction(180)
    elif command == "left":
        move_in_direction(270)
    elif command == "right":
        move_in_direction(90)
    elif command == "get_speeds":
        main_board.write_speeds()
    elif command == "toggle_red_led":
        main_board.toggle_red_led()


def movement_listener():
    rospy.init_node("movement_listener")
    rospy.Subscriber("movement", String, movement_callback)

    rospy.spin()


if __name__ == "__main__":
    movement_listener()
