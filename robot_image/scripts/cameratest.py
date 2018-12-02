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

import numpy as np
import cv2
import pyrealsense2 as rs
import select
import sys
import time

# Constants: TODO put this in a conf file
WIDTH = 1280
HEIGHT = 720
FRAME_RATE = 30

MIN_CONTOUR_AREA = 500
DEBUG = True

(lh, ls, lv) =(120, 50, 80)
(uh, us, uv) =(180, 100, 255)

# These aren't actually calibrated yet
BLUE_BASKET_LOWER_BOUND = np.array([90, 100, 40])
BLUE_BASKET_UPPER_BOUND = np.array([125, 255, 255])
# print cv2.cvtColor(np.uint8([[[6, 91, 66]]]), cv2.COLOR_BGR2HSV)

def debug_log(text):
    if DEBUG:
        print text

class ImageProcessor():
    def __init__(self):
        self.balls_in_frame = []

    def run(self):
        self.pipeline = rs.pipeline()
        print "got pipeline"
        config = rs.config()
        # TODO: do we want higher res? Or higher fps? Or both?
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FRAME_RATE)
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FRAME_RATE)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        print "starting"
        self.pipeline.start(config)

        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            if dev.supports(rs.camera_info.product_id) and dev.supports(rs.camera_info.name):
                print("Found camera device: {}".format(dev.get_info(rs.camera_info.name)))
                sensors = dev.query_sensors()
                for sensor in sensors:
                    if sensor.get_info(rs.camera_info.name) == "RGB Camera" and sensor.supports(
                            rs.option.exposure) and sensor.supports(rs.option.gain):
                        print("Setting RGB camera sensor settings")
                        sensor.set_option(rs.option.enable_auto_exposure, 1)
                        sensor.set_option(rs.option.white_balance, 3000)
                        sensor.set_option(rs.option.enable_auto_white_balance, 0)
                        print("exposure: {}".format(sensor.get_option(rs.option.exposure)))
                        print("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        print("gain: {}".format(sensor.get_option(rs.option.gain)))
                        print(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        time.sleep(2)
                        sensor.set_option(rs.option.enable_auto_exposure, 0)
                        print(
                            "auto exposure enabled: {}".format(sensor.get_option(rs.option.enable_auto_exposure)))
                        print("exposure: {}".format(sensor.get_option(rs.option.exposure)))  # 166
                        print("white balance: {}".format(sensor.get_option(rs.option.white_balance)))
                        print("gain: {}".format(sensor.get_option(rs.option.gain)))  # 64
                        break
                break

        print "started"


    def ball_color_recognizer(self, h, s, v):
        (lh, ls, lv) = BALL_COLOR_LOWER_BOUND
        (hh, hs, hv) = BALL_COLOR_UPPER_BOUND
        correct_hue = h >= lh and h <= hh
        correct_saturation = s >= ls and s <= hs
        correct_value = v >= lv and v <= hv
        return correct_hue and correct_saturation and correct_value
        
    def process_image(self):
        BALL_COLOR_LOWER_BOUND = np.array([lh, ls, lv])
        BALL_COLOR_UPPER_BOUND = np.array([uh, us, uv])

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        ball_mask = cv2.inRange(hsv_image, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)

        if DEBUG:
            color_image[HEIGHT / 2, WIDTH / 2] = [0, 0, 255]
            cv2.namedWindow('Test', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Test', color_image)
            cv2.namedWindow('2', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('2', ball_mask)
            cv2.namedWindow('hue', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('hue', hsv_image)
            cv2.waitKey(1)
            # raw_input()

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

    #def mouse_callback( event, x, y, flags, params):
     #   if event == 2:
     #       points.append(hsv[y][x])

    # def print_mask(self, mask):
    #     coverage = [0]*64
    #     for y in range(480):
    #         for x in range(640):
    #             c = mask[y, x]
    #             if c > 128:
    #                 coverage[x//10] += 1
            
    #         if y%20 is 19:
    #             line = ""
    #             for c in coverage:
    #                 line += " .:nhBXWW"[c//25]
    #             coverage = [0]*64
    #             print(line)


if __name__ == "__main__":
	def mouse_callback(event, x, y, flags, params):
	    #print "did sth"
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append(hsv[y][x])
		print "got point"

	print "start"
	camera = ImageProcessor()
	camera.run()

	cap = camera
	hsv = None
	points = list()

	

	#cv2.namedWindow('image')
	#cv2.namedWindow('tresh')
	#cv2.setMouseCallback('image', mouse_callback)

	while True:
	    points = list()

	    cv2.namedWindow('image')
	    cv2.namedWindow('tresh')
	    cv2.setMouseCallback('image', mouse_callback)

	    while True:

		frames = cap.pipeline.wait_for_frames()
		aligned_frames = cap.align.process(frames)
		color_frame = aligned_frames.get_color_frame()

		color_image = np.asanyarray(color_frame.get_data())
		hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)


		#blurred = cv2.GaussianBlur(frame, (5, 5), 0)
		#print "got frame"

		cv2.imshow('image', color_image)

		if len(points)>= 20:
		    print "enough"
		    break

		if cv2.waitKey(1) & 0xFF == ord('q'):
		    print "got frame"
		    break

	    #cap.release()
	    cv2.destroyWindow('image')
	    h = list()
	    s = list()
	    v = list()

	    for i in range(len(points)):
		h.append(points[i][0])
		s.append(points[i][1])
		v.append(points[i][2])

	    parameters = [min(h)-10, min(s)-10, min(v)-10, max(h)+10, max(s)+10, max(v)+10]
	    name = ""

	    LOWER_BOUND = np.array([min(h)-10, min(s)-10, min(v)-10])
            UPPER_BOUND = np.array([max(h)+10, max(s)+10, max(v)+10])

	    mask = cv2.inRange(hsv, LOWER_BOUND, UPPER_BOUND)
	    cv2.imshow('tresh', mask)

	    while name == "":
		objekt = input("Are those ball(1), blue basket(2) or magenta baske (3)parameters?")
		if objekt == 1:
		    name = "ball.txt"
		elif objekt == 2:
		    name = "blue_basket.txt"
		elif objekt == 3:
		    name = "magneta_basket.txt"
		elif objekt == 4:
		    name = "field.txt"
		else:
		    print("Wrong input, try again")

	    fail = open(name, "w")
	    for i in parameters:
	        fail.write(str(i) + ",")
	    fail.close()

	    uuesti = input("Would you like to do a new calibration?(1/0) ")
	    if uuesti == 0:
		break
	    else:
		continue
	
	    cv2.destroyAllWindows()

	old = '''while True:
        try:
            if select.select([sys.stdin,],[],[],0.0)[0]:
                change = raw_input()
                if change == "show":
                    print "({}, {}, {})\n({}, {}, {})".format(lh, ls, lv, uh, us, uv)
                num = int(change[2:])
                if change.startswith("lh"):
                    lh = num
                if change.startswith("ls"):
                    ls = num
                if change.startswith("lv"):
                    lv = num
                if change.startswith("uh"):
                    uh = num
                if change.startswith("us"):
                    us = num
                if change.startswith("uv"):
                    uv = num
        except:
            pass
        camera.process_image() '''

