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
from keyboard_publisher.msg import KeyEvent

pub = rospy.Publisher("movement", String, queue_size=10)

def key_event_callback(key_event):
    command = key_event.char
    translation_dict = { "w": "forward", "s": "backward", "a": "left",
                         "d": "right", "g": "get_speeds", "r":
                         "toggle_red_led" }
    if command in translation_dict:
        translated = translation_dict[command]
        pub.publish(translated)
    else:
        rospy.loginfo("Unrecognised keypress")
        
    

def key_listener():
    rospy.init_node("movement_controller")
    sub = rospy.Subscriber("keyboard_publisher/key_event", KeyEvent, key_event_callback)

    rospy.spin()


if __name__ == "__main__":
    try:
        key_listener()
    except rospy.ROSInterruptException:
        pass
