#!/usr/bin/env python
# image_processor.py --- Reads from the camera and writes object
# positions to the objects channel

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
import numpy as np
import cv2

# Constants: TODO put this in a conf file
WIDTH = 640
HEIGHT = 480


class ImageProcessor():
    def __init__(self):
        self.object_publisher = rospy.Publisher("image_processing/objects")

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        # TODO: do we want higher res? Or higher fps? Or both?
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 60)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.pipeline.start(config)

    def process_image():
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        ball_mask = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint8)
        ball_mask[color_image == ballcolor] = 255 # TODO: how would
        # this actually work? Advance boolean array indexing with
        # what? See
        # https://docs.scipy.org/doc/numpy/reference/arrays.indexing.html#arrays-indexing

        self.find_balls(ball_mask)

    def find_balls(mask):
        img, contour, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)



if __name__ == "__main__":
    try:
        camera = ImageProcessor()
        camera.run()
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
