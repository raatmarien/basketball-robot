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
import pyrealsense2 as rs

# Constants: TODO put this in a conf file
WIDTH = 640
HEIGHT = 480
FRAME_RATE = 30

MIN_CONTOUR_AREA = 100 # Should probably be more
DEBUG = True

BALL_COLOR_LOWER_BOUND = (30, 70, 62)
BALL_COLOR_UPPER_BOUND = (80, 255, 255)


def debug_log(text):
    if DEBUG:
        rospy.loginfo(text)

class ImageProcessor():
    def __init__(self):
        self.object_publisher = rospy.Publisher(
            "image_processing/objects", String, queue_size=10)
        self.balls_in_frame = []

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        # TODO: do we want higher res? Or higher fps? Or both?
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FRAME_RATE)
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FRAME_RATE)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.pipeline.start(config)

    def ball_color_recognizer(self, h, s, v):
        (lh, ls, lv) = BALL_COLOR_LOWER_BOUND
        (hh, hs, hv) = BALL_COLOR_UPPER_BOUND
        correct_hue = h >= lh and h <= hh
        correct_saturation = s >= ls and s <= hs
        correct_value = v >= lv and v <= hv
        return correct_hue and correct_saturation and correct_value
        
    def process_image(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        ball_mask = np.zeros((HEIGHT, WIDTH, 1), dtype=np.uint8)

        for y in range(0, HEIGHT):
            for x in range(0, WIDTH):
                [h, s, v] = hsv_image[y, x]
                ball_mask[y, x, 0] = 255 if self.ball_color_recognizer(h, s, v) else 0

        self.find_balls(ball_mask)

        debug_log(str(len(self.balls_in_frame)) + " balls found")

        for ball in self.balls_in_frame:
            self.object_publisher.publish(str(ball))

        if DEBUG:
            rospy.loginfo(str(self.balls_in_frame))
            self.print_mask(ball_mask)

    def find_balls(self, mask):
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

        self.balls_in_frame = []

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size < MIN_CONTOUR_AREA:
                continue

            rect = cv2.boundingRect(contour)

            self.balls_in_frame.append(rect)

    def print_mask(self, mask):
        coverage = [0]*64
        for y in range(480):
            for x in range(640):
                c = mask[y, x]
                if c > 128:
                    coverage[x//10] += 1
            
            if y%20 is 19:
                line = ""
                for c in coverage:
                    line += " .:nhBXWW"[c//25]
                coverage = [0]*64
                rospy.loginfo(line)


if __name__ == "__main__":
    try:
        rospy.init_node("image_processor")
        camera = ImageProcessor()
        camera.run()
        rate = rospy.Rate(FRAME_RATE)

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
