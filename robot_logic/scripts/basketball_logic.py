#!/usr/bin/env python
# basketball_logic.py -- This node should do the main logic of the
# robot as described in the state diagram. It assumes we want to score in the
# blue basket now.

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
import time
from ast import literal_eval

# Constants
CENTER_REGION = 0.03
CENTER_BASKET_REGION = 0.03


TIME_MOVING_FORWARD = 8
TIME_THROWING = 2
DISTANCE_TO_CENTER_BALL = 0.5
CAMERA_FOV = 29.0 # Trial and error (mostly error though)


# Variable
DEBUG = False
SCORE_IN_BLUE = False

 # Change to False if we need to score in the
                     # magenta basket


class State:
    STOPPED, FORWARD, TURNING, MOVING_AND_TURNING, \
        CENTERING, THROWING = range(6)


def log(text):
    TAG = "robot_logic/basketball_logic"
    if DEBUG:
        rospy.loginfo(TAG + ": " + text)

class BasketballLogic:
    def __init__(self):
        self.turn_left = True
        self.movement_publisher = rospy.Publisher("movement", String, queue_size=10)
        self.move_to_state(State.STOPPED)
        self.basket_distances = [0]
        self.basket_last_seen_left = True

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)

    def react(self, position, baskets):
        log("Reacting in state {}".format(self.state))

        self.update_basket_info(baskets)
        
        if self.state == State.STOPPED:
            self.react_stopped(position, baskets)
        elif self.state == State.FORWARD:
            self.react_forward(position, baskets)
        elif self.state == State.TURNING:
            self.react_turning(position, baskets)
        elif self.state == State.MOVING_AND_TURNING:
            self.react_moving_and_turning(position, baskets)
        elif self.state == State.CENTERING:
            self.react_centering(position, baskets)
        elif self.state == State.THROWING:
            self.react_throwing(position, baskets)

    def update_basket_info(self, baskets):
        if SCORE_IN_BLUE:
            basket_message = baskets.split(":")[0]
        else:
            basket_message = baskets.split(":")[1]
        basket_msg = literal_eval(basket_message)
        if basket_msg is not None:
            (basket, distance) = basket_msg
            (bx, by, bw, bh) = basket
            horizontal_basket_position = bx + (bw / 2.0) - 0.5
            self.basket_distances.append(distance)
            if len(self.basket_distances) > 5:
                self.basket_distances = self.basket_distances[1:]
            self.basket_last_seen_left = horizontal_basket_position < 0
            
    def react_stopped(self, position, baskets):
        self.send("stop")

    def react_forward(self, position, baskets):
        log("In state forward")
        if (time.time() - self.forward_start_time) > TIME_MOVING_FORWARD:
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        self.send("movement:75:0:0")

    def react_turning(self, position, baskets):
        log("In state turning")
        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        if self.turn_left:
            self.send("movement:0:0:-80")
        else:
            self.send("movement:0:0:80")

    def react_moving_and_turning(self, position, baskets):
        log("In state moving and turning")
        if position == "None":
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        pos = float(position.split(":")[0])
        distance = float(position.split(":")[1])

        if distance < DISTANCE_TO_CENTER_BALL and pos < 0.03 and pos > -0.03:
            self.move_to_state(State.CENTERING)
            self.react(position, baskets)
            return

        rotational_speed = max(min(pos, 0.1), -0.1) * 800

        if distance < DISTANCE_TO_CENTER_BALL:
            speed = 0
        else:
            speed = 60 * abs(max(min(distance, 1.5), -1.5))

        self.send("movement:{}:{}:{}".format(speed, pos * CAMERA_FOV,
                                             rotational_speed))

    def react_centering(self, position, baskets):
        log("In state centering")
        if position == "None":
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        pos = float(position.split(":")[0])

        if SCORE_IN_BLUE:
            basket_message = baskets.split(":")[0]
        else:
            basket_message = baskets.split(":")[1]
        log("Basket: {}".format(basket_message))
        basket_msg = literal_eval(basket_message)
        if basket_msg is None:
            log("Basket not found")
            if self.basket_last_seen_left:
                horizontal_basket_position = -1.0
            else:
                horizontal_basket_position = 1.0
        else:
            (basket, distance) = basket_msg
            (bx, by, bw, bh) = basket
            horizontal_basket_position = bx + (bw / 2.0) - 0.5
            log("Basket position is {}".format(horizontal_basket_position))

        if horizontal_basket_position >= -CENTER_BASKET_REGION + 0.01  and \
           horizontal_basket_position <= CENTER_BASKET_REGION - 0.01 and \
           pos < 0.01 and pos > -0.01:
            self.move_to_state(State.THROWING)
            self.react(position, baskets)
            return

        sideways_slowdown = 0.2
        sideways_speed = 16 * (max(-sideways_slowdown,
                                   min(sideways_slowdown,
                                       horizontal_basket_position)) \
                               / sideways_slowdown) + 1

        turn_slowdown = 0.1
        turn_speed = 70 * (max(-turn_slowdown,
                                 min(turn_slowdown, pos)) \
                             / turn_slowdown)+1

        self.send("movement:{}:-90:{}".format(sideways_speed, turn_speed))

    def react_throwing(self, position, baskets):
        log("In state throwing")
        if (time.time() - self.throwing_start_time) > TIME_THROWING:
            self.move_to_state(State.TURNING)
            self.send("throw:50")
            self.react(position, baskets)
            return

        self.throw()

    def move_to_state(self, state):
        log("Moving to state {}".format(state))
        if state == State.FORWARD:
            self.forward_start_time = time.time()
        elif state == State.THROWING:
            self.throwing_start_time = time.time()
        self.state = state

    def throw(self):
        self.send("forward")
	rate = rospy.Rate(100)
	rate.sleep()
        average_distance = sum(self.basket_distances) / len(self.basket_distances)
        speed = self.get_throw_speed(average_distance)
        self.send("throw:{}".format(int(round(speed))))

    def get_throw_speed_f(self, distance):
	inertia_const = 30
	speed = int(-2*math.pow(10,-18)*math.pow(distance,6)+1*math.pow(10,-14)*math.pow(distance,5)+2*math.pow(10,-11)*math.pow(distance,4)-2*math.pow(10,-7)*math.pow(distance,3)+0.0005*math.pow(distance,2)-0.4037*distance+279.09)
	return speed-inertia_const

    def get_throw_speed(self, distance):
	movement_constant = -2
	dist_const = 0

	distance = (distance + dist_const)
	print distance
        
	#speeds = [(0.75, 160), (2.0, 165)]
        #speeds = [(0.76, 172),
        #           (1.157, 175),
        #           (1.52, 180),
        #           (2.44, 188),
        #           (2.61, 240),
        #           (3.42, 270)]
	speeds = [(0.6,145),
		    (0.8,145),
		    (1.0,150),
		    (1.2,150),
		    (1.4,154),
		    (1.6,160),
		    (1.8,165),
		    (2.0,179),
		    (2.2,183),
		    (2.4,185),
		    (2.6,195),
		    (2.8,207),
		    (3.0,219),
		    (3.2,233),
		    (3.4,245),
		    (3.6,260),
		    (3.8,270),
		    (4.0,270),
		    (4.2,270)]

        for i in range(len(speeds)):
	    if distance<= speeds[0][0]:
	
		return float(speeds[0][1])
	    if distance < speeds[i][0]:
		#return float(speeds[i][1]-(speeds[i][0]-distance)*(speeds[i][1]-speeds[i-1][1])/0.2 - movement_constant)*1.65 + 25
		return float(speeds[i][1]-(speeds[i][0]-distance)*(speeds[i][1]-speeds[i-1][1])/0.2 - movement_constant)
		

	#(min_dist, min_speed) = speeds[0]
        #(sec_dist, sec_speed) = speeds[1]
        #(sm_dist, sm_speed) = speeds[len(speeds) - 2]
        #(max_dist, max_speed) = speeds[len(speeds) - 1]
        #if distance <= min_dist:
        #    speed_per_dist = (sec_speed - min_speed) / (sec_dist - min_dist)
        #    return min_speed - ((min_dist - distance) * speed_per_dist)-movement_constant
        #elif distance >= max_dist:
        #    speed_per_dist = (max_speed - sm_speed) / (max_dist - sm_dist)
        #    return max_speed + ((distance - max_dist) * speed_per_dist)-movement_constant
        #else:
        #    (prev_dist, prev_speed) = speeds[0]
        #    for (cur_dist, cur_speed) in speeds[1:]:
        #        if distance < cur_dist:
        #            speed_per_dist = (cur_speed - prev_speed) / (cur_dist - prev_dist)
        #            return prev_speed + ((distance - prev_dist) * speed_per_dist)-movement_constant


logic = BasketballLogic()

def new_object_callback(message):
    logic.react(message.data.split("\n")[0],
                  message.data.split("\n")[1])


def new_referee_command_callback(message):
    if message.data == "start" and logic.state == State.STOPPED:
        logic.move_to_state(State.FORWARD)
    elif message.data == "stop":
        logic.move_to_state(State.STOPPED)


if __name__ == "__main__":
    rospy.init_node("basketball_logic")
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    rospy.Subscriber("robot_main/referee", String, new_referee_command_callback)

    rospy.spin()
