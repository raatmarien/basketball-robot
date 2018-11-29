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
import time

# Constants: TODO put this in a conf file
WIDTH = 1280
HEIGHT = 720
FRAME_RATE = 30

MIN_BALL_CONTOUR_AREA = 30
MIN_BASKET_CONTOUR_AREA = 100
MAX_BALL_CONTOUR_AREA = 2000
DEBUG = False

# Important reading about cv2 color spaces:
# https://docs.opencv.org/3.4.2/df/d9d/tutorial_py_colorspaces.html
# Hue goes from 0 to 179

ball_fail = open("/home/robot/catkin_ws/src/robot_image/scripts/ball.txt",'r')
ball_values = ball_fail.read().split(',')
BALL_COLOR_LOWER_BOUND = np.array([int(ball_values[0]), int(ball_values[1]), int(ball_values[2])])
BALL_COLOR_UPPER_BOUND = np.array([int(ball_values[3]), int(ball_values[4]), int(ball_values[5])])
ball_fail.close()

blue_fail = open("/home/robot/catkin_ws/src/robot_image/scripts/blue_basket.txt",'r')
blue_values = blue_fail.read().split(',')
BLUE_BASKET_LOWER_BOUND = np.array([int(blue_values[0]), int(blue_values[1]), int(blue_values[2])])
BLUE_BASKET_UPPER_BOUND = np.array([int(blue_values[3]), int(blue_values[4]), int(blue_values[5])])
blue_fail.close()

mage_fail = open("/home/robot/catkin_ws/src/robot_image/scripts/magneta_basket.txt",'r')
mage_values = mage_fail.read().split(',')
MAGENTA_BASKET_LOWER_BOUND = np.array([int(mage_values[0]), int(mage_values[1]), int(mage_values[2])])
MAGENTA_BASKET_UPPER_BOUND = np.array([int(mage_values[3]), int(mage_values[4]), int(mage_values[5])])
mage_fail.close()

#BLUE_BASKET_LOWER_BOUND = np.array([100, 110, 70])
#BLUE_BASKET_UPPER_BOUND = np.array([130, 250, 140])
#MAGENTA_BASKET_LOWER_BOUND = np.array([160, 150, 150])
#MAGENTA_BASKET_UPPER_BOUND = np.array([180, 180, 180])

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
        self.profile = self.pipeline.start(config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and dev.supports(rs.camera_info.name):
                rospy.loginfo("Found camera device: {}".format(dev.get_info(rs.camera_info.name)))
                sensors = dev.query_sensors()
                for sensor in sensors:
                    if sensor.get_info(rs.camera_info.name) == "RGB Camera" and sensor.supports(
                            rs.option.exposure) and sensor.supports(rs.option.gain):
                        rospy.loginfo("Setting RGB camera sensor settings")
                        sensor.set_option(rs.option.enable_auto_exposure, 1)
                        sensor.set_option(rs.option.white_balance, 3000)
                        sensor.set_option(rs.option.enable_auto_white_balance, 0)
                        rospy.loginfo("exposure: {}".format(sensor.get_option(rs.option.exposure)))
                        rospy.loginfo("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        rospy.loginfo("gain: {}".format(sensor.get_option(rs.option.gain)))
                        rospy.loginfo(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        time.sleep(2)
                        sensor.set_option(rs.option.enable_auto_exposure, 0)
                        rospy.loginfo(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        rospy.loginfo("exposure: {}".format(sensor.get_option(rs.option.exposure)))  # 166
                        rospy.loginfo("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        rospy.loginfo("gain: {}".format(sensor.get_option(rs.option.gain)))  # 64
                        break
                break
        
    def process_image(self):
        # rospy.loginfo("Image: scanning frame")
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        ball_mask = cv2.inRange(hsv_image, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)
        self.find_balls(ball_mask, depth_image)

        blue_basket_mask = cv2.inRange(hsv_image,
                                       BLUE_BASKET_LOWER_BOUND,
                                       BLUE_BASKET_UPPER_BOUND)
        self.blue_basket = self.find_basket(blue_basket_mask, depth_image)

        magenta_basket_mask = cv2.inRange(hsv_image,
                                          MAGENTA_BASKET_LOWER_BOUND,
                                          MAGENTA_BASKET_UPPER_BOUND)
        self.magenta_basket = self.find_basket(magenta_basket_mask, depth_image)

        self.send_objects()

        if self.blue_basket is not None:
            (d, distance) = self.blue_basket
            if DEBUG:
                rospy.loginfo("Distance from blue basket is {}".format(distance))
        elif DEBUG:
            rospy.loginfo("No basket seen")

        if DEBUG:
            rospy.loginfo(str(self.balls_in_frame))
            self.print_mask(ball_mask)

    def find_balls(self, mask, depth):
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

        self.balls_in_frame = []

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size < MIN_BALL_CONTOUR_AREA:
                continue

	    #if contour_size > MAX_BALL_CONTOUR_AREA:
            #    continue

            rect = cv2.boundingRect(contour)

	    if rect[1] < 100:
	    	continue

            distance = self.get_corrected_mean_distance(contour, depth)

            self.balls_in_frame.append((rect, distance))

    def get_corrected_mean_distance(self, contour, depth):
        cimg = np.zeros_like(depth)
        cv2.drawContours(cimg, [contour], -1, color=255, thickness=-1)

        # Access the image pixels and create a 1D numpy array then add to list
        pts = np.where(cimg == 255)
        distances = []
        distances.append(depth[pts[0], pts[1]])

        dists = np.array(distances)
        nz = dists > 0
        nzd = dists[nz]

        if nzd.size == 0:
            return 0
        
        distance = np.mean(nzd)

        return distance * self.depth_scale
	#return distance

    def find_basket(self, mask, depth):
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        basket = None
        biggest_contour_size = MIN_BASKET_CONTOUR_AREA
        dist = None

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size > biggest_contour_size:
                biggest_contour_size = contour_size
                dist = self.get_corrected_mean_distance(contour, depth) 
                basket = cv2.boundingRect(contour)

        if basket is None:
            return None

        (sx, sy, sw, sh) = basket
            
        wf = float(WIDTH)
        hf = float(HEIGHT)
        # In relative coordinates
        return ((sx / wf, sy / hf, sw / wf, sh / hf), dist)

    def get_largest_ball(self):
        debug_log(str(len(self.balls_in_frame)) + " balls found")

        # Only send the horizontal location of the biggest ball for
        # now
        if len(self.balls_in_frame) > 0:
            max_size = 0
            max_ball = ()
            max_ball_distance = 0
            for ((x, y, width, height), distance) in self.balls_in_frame:
		#if y < 100:
	    	#    continue
                if (width * height) > max_size:
                    max_size = width * height
                    max_ball = (x, y, width, height)
                    max_ball_distance = distance
            (x, y, width, height) = max_ball
            ball_pos = str(((x + (width / 2)) / float(WIDTH)) - 0.5)
            debug_log("Ball at " + ball_pos)
            return ball_pos + ":" + str(max_ball_distance)
        else:
            return "None"

    def get_closest_ball(self):
	debug_log(str(len(self.balls_in_frame)) + " balls found")

        # Only send the y location of the biggest ball for
        # now
        if len(self.balls_in_frame) > 0:
	    closest_y = 0
            closest_ball = ()
	    closest_ball_distance = 0
	    for ((x, y, width, height), distance) in self.balls_in_frame:
                if y > closest_y:
                    closest_y = y
                    closest_ball = (x, y, width, height)
                    closest_ball_distance = distance
            (x, y, width, height) = closest_ball
            ball_pos = str(((x + (width / 2)) / float(WIDTH)) - 0.5)
            debug_log("Ball at " + ball_pos)
            return ball_pos + ":" + str(closest_ball_distance)

    def send_objects(self):
	#ball = self.get_closest_ball()
        ball = self.get_largest_ball()
        baskets = "{}:{}".format(self.blue_basket, self.magenta_basket)
        message = "{}\n{}".format(ball, baskets)
        self.object_publisher.publish(message)
        
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
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
