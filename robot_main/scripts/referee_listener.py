#!/usr/bin/env python
# referee_listener -- listens to replies from the main board and
# sends referee commmands

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

movement_pub = rospy.Publisher("movement", String, queue_size=10)
referee_pub = rospy.Publisher("robot_main/referee", String, queue_size=10)

robotID = "A"
fieldID = "A"

def mainboard_callback(message):
    if len(message.data.split(":")) == 1:
        return
    without_ref = message.data.split(":")[1]
    if len(without_ref) > 1:
        command = without_ref[:-1]
        if command == "a" + fieldID + "XSTART----" or command == "a" + fieldID + robotID + "START----":
            referee_pub.publish("start")
        elif command == "a" + fieldID + "XSTOP-----" or command == "a" + fieldID + robotID + "STOP-----":
            referee_pub.publish("stop")
        elif command ==  'a' + fieldID + robotID + 'PING-----':
            movement_pub.publish("ping:{}:{}".format(robotID, fieldID))

if __name__ == "__main__":
    try:
        rospy.init_node("referee_listener")
        rospy.Subscriber("mainboard", String, mainboard_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
