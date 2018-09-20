#!/usr/bin/env python
# movement_controller.py --- Sends drive commands to the hardware
# module

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

import rospy
from std_msgs.msg import String

def movement_controller():
    pub = rospy.Publisher("movement", String, queue_size=10)
    rospy.init_node("movement_controller")
    rate = rospy.Rate(30)

    # Let's move the robot manually for now
    while not rospy.is_shutdown():
        command = raw_input()

        translation_dict = { "w": "forward", "s": "backward", "a":
                             "left", "d": "right", "g": "get_speeds",
                             "r": "toggle_red_led", "p": "stop", "l":
                             "turn_left", "t": "throw" }

        if command in translation_dict:
            pub.publish(translation_dict[command])
        else:
            rospy.loginfo("Invalid command typed!")

        rate.sleep()


if __name__ == "__main__":
    try:
        movement_controller()
    except rospy.ROSInterruptException:
        pass
