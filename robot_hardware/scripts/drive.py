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
WHEEL_ONE_ANGLE = 60
WHEEL_TWO_ANGLE = 270
WHEEL_THREE_ANGLE = 120

class Driver:
    def __init__(self):
        self.main_board = ComportMainboard()
        self.main_board.run()

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0

    def get_relative_speed_for_wheel(self, wheel_angle, drive_angle):
        return math.cos(math.radians(drive_angle - wheel_angle))

    def move_in_direction(self, degrees):
        speed = 10

        w1 = speed * self.get_relative_speed_for_wheel(WHEEL_ONE_ANGLE, degrees)
        w2 = speed * self.get_relative_speed_for_wheel(WHEEL_TWO_ANGLE, degrees)
        w3 = speed * self.get_relative_speed_for_wheel(WHEEL_THREE_ANGLE, degrees)

        self.set_wheels(round(w1, 2), round(w2, 2), round(w3, 2))

    def set_wheels(self, w1, w2, w3):
        rospy.loginfo("Changing wheels")
        self.wheel_one_speed = w1
        self.wheel_two_speed = w2
        self.wheel_three_speed = w3
        self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)
    

    def movement_callback(self, command):
        command = str(command)[7:][:-1]
        rospy.loginfo("Recieved command: " + command)
        if command == "forward":
            self.move_in_direction(0)
        elif command == "backward":
            self.move_in_direction(180)
        elif command == "left":
            self.move_in_direction(270)
        elif command == "right":
            self.move_in_direction(90)
        elif command == "get_speeds":
            self.main_board.write_speeds()
        elif command == "toggle_red_led":
            self.main_board.toggle_red_led()
        elif command == "stop":
            self.set_wheels(0, 0, 0)
        elif command == "turn_left":
            self.set_wheels(-1, -1, -1)
        elif command == "turn_right":
            self.set_wheels(1, 1, 1)

    def movement_listener(self):
        rospy.init_node("movement_listener")
        rospy.Subscriber("movement", String, self.movement_callback)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)

            rate.sleep()

        self.main_board.close()


if __name__ == "__main__":
    try:
        drive = Driver()
        drive.movement_listener()
    except rospy.ROSInterruptException:
        pass
