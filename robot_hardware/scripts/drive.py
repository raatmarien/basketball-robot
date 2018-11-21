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
from time import sleep

# Constants
# Wheel angles in degrees
WHEEL_ONE_ANGLE = 150
WHEEL_TWO_ANGLE = 270
WHEEL_THREE_ANGLE = 30
WHEEL_DISTANCE_FROM_CENTER = 0.133
ROBOT_SPEED = 30
ROBOT_TURN_SPEED = 50

class Driver:
    def __init__(self):
        self.main_board = ComportMainboard()
        self.main_board.run()

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0
        self.throw_speed = 100


    def get_speed_for_wheel(self, wheel_angle, drive_angle,
                            robot_speed, wheel_distance_from_center,
                            robot_angular_velocity):
        move_speed = robot_speed * math.cos(math.radians(drive_angle -
                            wheel_angle))
        turn_speed = wheel_distance_from_center * robot_angular_velocity
        return move_speed + turn_speed

    def set_movement(self, linear_speed, direction_degrees, angular_speed):
        w1 = self.get_speed_for_wheel(WHEEL_ONE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w2 = self.get_speed_for_wheel(WHEEL_TWO_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w3 = self.get_speed_for_wheel(WHEEL_THREE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)

        self.set_wheels(int(round(w1)), int(round(w2)), int(round(w3)))

    def set_wheels(self, w1, w2, w3):
        if w1 != self.wheel_one_speed or w2 != self.wheel_two_speed or w3 != self.wheel_three_speed:
            self.wheel_one_speed = w1
            self.wheel_two_speed = w2
            self.wheel_three_speed = w3

            self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)


    def movement_callback(self, command):
        command = str(command)[7:][:-1]
        if command == "forward":
            self.set_movement(ROBOT_SPEED, 0, 0)
        elif command == "backward":
            self.set_movement(ROBOT_SPEED, 180, 0)
        elif command == "left":
            self.set_movement(ROBOT_SPEED, 270, 0)
        elif command == "right":
            self.set_movement(ROBOT_SPEED, 90, 0)
        elif command == "get_speeds":
            self.main_board.write_speeds()
        elif command == "toggle_red_led":
            self.main_board.toggle_red_led()
        elif command == "stop":
            self.set_movement(0, 0, 0)
        elif command == "turn_left":
            self.set_movement(0, 0, -ROBOT_TURN_SPEED)
        elif command == "turn_right":
            self.set_movement(0, 0, ROBOT_TURN_SPEED)
        elif command.split(":")[0] == "throw":
            try:
                splitted = command.split(":")
                if int(splitted[1]) != int(self.throw_speed):
                    self.throw_speed = splitted[1]
                    self.main_board.set_throw(self.throw_speed)
            except:
                rospy.loginfo("Incorrect throw command received and ignored")
        elif command.split(":")[0] == "movement":
            try:
                splitted = command.split(":")
                self.set_movement(int(round(float(splitted[1]))),
                                  int(round(float(splitted[2]))),
                                  int(round(float(splitted[3]))))
            except:
                rospy.loginfo("Incorrect movement command received and ignored")
        elif command.split(":")[0] == "ping":
            try:
                rospy.loginfo("Sending ping command")
                splitted = command.split(":")
                self.main_board.send_ping(splitted[1], splitted[2])
            except:
                rospy.loginfo("Incorrect ping command received and ignored")
        elif command.split(":")[0] == "servo":
            try:
                splitted = command.split(":")
                self.main_board.set_servo(int(splitted[1]))
            except:
                rospy.loginfo("Incorrect servo command received and ignored")

    def movement_listener(self):
        rospy.init_node("movement_listener")
        rospy.Subscriber("movement", String, self.movement_callback)
        self.mainboard_out = rospy.Publisher("mainboard", String, queue_size=10)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.main_board.set_wheels(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)
            self.main_board.set_throw(self.throw_speed)

            messages = self.main_board.read()
            for message in messages.split("\n"):
                if message != "":
                    rospy.loginfo("Sending hardware msg: " + message)
                    self.mainboard_out.publish(message)

            rate.sleep()

        self.main_board.close()


if __name__ == "__main__":
    try:
        drive = Driver()
        drive.movement_listener()
    except rospy.ROSInterruptException:
        pass
