#!/usr/bin/env python
# camera_shower.py --- Basic camera tests

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
import pyrealsense2 as rs

pipeline = rs.pipeline()
pipeline.start()

def screen_request_callback(command):
    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
    if not depth: return

    for i in range(0, 10):
        rospy.loginfo("x" * 64)
    
    coverage = [0]*64
    for y in range(480):
        for x in range(640):
            dist = depth.get_distance(x, y)
            if 0 < dist and dist < 1:
                coverage[x//10] += 1
            
        if y%20 is 19:
            line = ""
            for c in coverage:
                line += " .:nhBXWW"[c//25]
            coverage = [0]*64
            rospy.loginfo(line)


def camera_shower():
    rospy.init_node("camera_shower")
    rospy.Subscriber("screen_request", String, screen_request_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        camera_shower()
    except rospy.ROSInterruptException:
        pass
