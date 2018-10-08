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

pub = rospy.Publisher("referee", String, queue_size=10)


def mainboard_callback(message):
    rospy.loginfo(message.data)


if __name__ == "__main__":
    try:
        rospy.init_node("referee_listener")
        rospy.Subscriber("mainboard", String, mainboard_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
