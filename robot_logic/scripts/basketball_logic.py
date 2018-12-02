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
CENTER_REGION = 0.02
CENTER_BASKET_REGION = 0.01
CENTER_BALL_REGION = 0.01


TIME_THROWING = 2
TIME_BACKING_UP = 0.3
TIME_SEARCHING = 1.5
TIME_TURNING = 10
DISTANCE_TO_CENTER_BALL = 0.5
CAMERA_FOV = 29.0 # Trial and error (mostly error though)


# Variable
DEBUG = False
SCORE_IN_BLUE = True

 # Change to False if we need to score in the
                     # magenta basket


class State:
    STOPPED, FORWARD, TURNING, MOVING_AND_TURNING, \
        CENTERING, THROWING, BACKING_UP, SEARCHING = range(8)


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
        self.time_moving_forward = 2

    def send(self, command):
        log("Sending {}".format(command))
        self.movement_publisher.publish(command)

    def react(self, position, baskets):
        rospy.loginfo("Reacting in state {}".format(self.state))
        
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
        elif self.state == State.BACKING_UP:
            self.react_backing_up(position, baskets)
        elif self.state == State.SEARCHING:
            self.react_searching(position, baskets)

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
            if len(self.basket_distances) > 5:
                self.basket_distances = self.basket_distances[1:]
            self.basket_last_seen_left = horizontal_basket_position < 0
            if distance <= 0.001:
                return
            self.basket_distances.append(distance)
            rospy.loginfo("Distance is: {}".format(sum(self.basket_distances) / len(self.basket_distances)))
            
    def react_stopped(self, position, baskets):
        self.send("stop")

    def react_forward(self, position, baskets):
        log("In state forward")
        if (time.time() - self.forward_start_time) > self.time_moving_forward:
            self.time_moving_forward = 0
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        # if position != "None":
        #     self.move_to_state(State.MOVING_AND_TURNING)
        #     self.react(position, baskets)
        #     return

        self.send("movement:75:0:0")

    def react_turning(self, position, baskets):
        log("In state turning")
        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        if (time.time() - self.turning_start_time) > TIME_TURNING:
            self.move_to_state(State.SEARCHING)
            self.react(position, baskets)
            return

        if self.turn_left:
            self.send("movement:0:0:-100")
        else:
            self.send("movement:0:0:100")

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
            self.move_to_state(State.BACKING_UP)
            self.react(position, baskets)
            return

        pos = float(position.split(":")[0])
        ball_distance = float(position.split(":")[1])

        if ball_distance > DISTANCE_TO_CENTER_BALL + 0.1:
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

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
            basket_adjustment = -0.025
            horizontal_basket_position = bx + (bw / 2.0) - 0.5 + basket_adjustment
            log("Basket position is {}".format(horizontal_basket_position))

        if horizontal_basket_position >= -CENTER_BASKET_REGION  and \
           horizontal_basket_position <= CENTER_BASKET_REGION and \
           pos < CENTER_BALL_REGION and pos > -CENTER_BALL_REGION:
            self.send("movement:0:0:0")
	    self.move_to_state(State.THROWING)
            self.react(position, baskets)
            return

        sideways_slowdown = 0.45
        sideways_speed = 36 * (max(-sideways_slowdown,
                                   min(sideways_slowdown,
                                       horizontal_basket_position)) \
                               / sideways_slowdown)
        if sideways_speed > 0:
            sideways_speed += 1
        else:
            sideways_speed -= 1

        turn_slowdown = 0.15
        turn_speed = 144 * (max(-turn_slowdown,
                                 min(turn_slowdown, pos)) \
                             / turn_slowdown)

        if turn_speed > 0:
            turn_speed += 1
        else:
            turn_speed -= 1

        rospy.loginfo("Sideways {}\t Turn {}".format(sideways_speed, turn_speed))

        self.send("movement:{}:-90:{}".format(sideways_speed, turn_speed))

        self.throw(ball_distance)

    def react_throwing(self, position, baskets):
        log("In state throwing")
        if (time.time() - self.throwing_start_time) > TIME_THROWING:
            self.move_to_state(State.TURNING)
            self.send("throw:50")
            self.react(position, baskets)
            return

        if position != "None":
            distance = float(position.split(":")[1])
        else:
            distance = 0.0

        self.send("forward")
        self.throw(distance)

    def react_backing_up(self, position, baskets):
        log("In state backing up")
        if (time.time() - self.backing_up_start_time) > TIME_BACKING_UP:
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        self.send("backward")

    def react_searching(self, position, baskets):
        log("In state searching")
        if (time.time() - self.searching_start_time) > TIME_SEARCHING:
            self.move_to_state(State.TURNING)
            self.react(position, baskets)
            return

        if position != "None":
            self.move_to_state(State.MOVING_AND_TURNING)
            self.react(position, baskets)
            return

        self.send("backward")

    def move_to_state(self, state):
        log("Moving to state {}".format(state))
        if state == State.FORWARD:
            self.forward_start_time = time.time()
        elif state == State.THROWING:
            self.throwing_start_time = time.time()
        elif state == State.BACKING_UP:
            self.backing_up_start_time = time.time()
        elif state == State.SEARCHING:
            self.searching_start_time = time.time()
        elif state == State.TURNING:
            self.turning_start_time = time.time()
            
        self.state = state
        rate = rospy.Rate(50)
        rate.sleep()

    def throw(self, ball_distance):
	rate = rospy.Rate(30)
	rate.sleep()
        rospy.loginfo(str(self.basket_distances))
        average_distance = sum(self.basket_distances) / len(self.basket_distances)
        speed = self.get_throw_speed(average_distance)
        self.send("throw:{}".format(int(round(speed))))

    def get_throw_speed_f(self, distance):
	inertia_const = 30
	speed = int(-2*math.pow(10,-18)*math.pow(distance,6)+1*math.pow(10,-14)*math.pow(distance,5)+2*math.pow(10,-11)*math.pow(distance,4)-2*math.pow(10,-7)*math.pow(distance,3)+0.0005*math.pow(distance,2)-0.4037*distance+279.09)
	return speed-inertia_const

    def get_throw_speed(self, distance):
	movement_constant = 0
	dist_const = -0.19

        distance += dist_const

	#speeds = [(0.75, 160), (2.0, 165)]
        # speeds = [(0.76, 172),
        #           (1.157, 175),
        #           (1.52, 180),
        #           (2.44, 188),
        #           (2.61, 240),
        #           (3.42, 270)]
	# speeds = [(0.6,1130),
	# 	    (0.8,1140),
	# 	    (1.0,1160),
	# 	    (1.2,1170),
	# 	    (1.4,1193),
	# 	    (1.6,1233),
	# 	    (1.8,1267),
	# 	    (2.0,1360),
	# 	    (2.2,1387),
	# 	    (2.4,1400),
	# 	    (2.6,1467),
	# 	    (2.8,1547),
	# 	    (3.0,1627),
	# 	    (3.2,1720),
	# 	    (3.4,1800),
	# 	    (3.6,1900),
	# 	    (3.8,1967),
	# 	    (4.0,2000),
	# 	    (4.2,2000)]
        # speeds = [(0.75, 1390),
        #           (1.03, 1420),
        #           (1.19, 1460),
        #           (1.5, 1500),
        #           (2.0, 1550),
        #           (2.45, 1615),
        #           (3.0, 1940),
        #           (3.4, 2150)]
        # speeds = [(0.70, 175),
        #           (0.91, 180),
        #           (1.10, 182),
        #           (1.46, 187),
        #           (1.69, 190),
        #           (2.37, 200),
        #           (2.65, 230)]
        # speeds = [(2.36, 197),
        #           (1.42, 189),
        #           (1.18, 182)][::-1]

        # speeds = [(0.68, 177),
        #           (1.0, 181),
        #           (1.35, 187),
        #           (1.87, 196),
        #           (2.2, 213)]

        speeds = [(0.604, 175),
                  (0.76, 178),
                  (0.869, 180),
                  (0.978, 182),
                  (1.088, 184),
                  (1.193, 186),
                  (1.328, 188),
                  (1.422, 189),
                  (1.538, 191),
                  (1.675, 193),
                  (1.828, 195),
                  (2.012, 198),
                  (2.098, 199),
                  (2.263, 202),
                  (2.33, 208),
                  (2.43, 211),
                  (2.62, 230),
                  (2.71, 243),
                  (3.33, 260),
                  (3.6, 270)]

        # return 

        # for i in range(len(speeds)):
	#     if distance<= speeds[0][0]:
	
	# 	return float(speeds[0][1])
	#     if distance < speeds[i][0]:
	# 	#return float(speeds[i][1]-(speeds[i][0]-distance)*(speeds[i][1]-speeds[i-1][1])/0.2 - movement_constant)*1.65 + 25
	# 	return float(speeds[i][1]-(speeds[i][0]-distance)*(speeds[i][1]-speeds[i-1][1])/0.2 - movement_constant)
	# return 2000
		

	(min_dist, min_speed) = speeds[0]
        (sec_dist, sec_speed) = speeds[1]
        (sm_dist, sm_speed) = speeds[len(speeds) - 2]
        (max_dist, max_speed) = speeds[len(speeds) - 1]
        if distance <= min_dist:
           speed_per_dist = (sec_speed - min_speed) / (sec_dist - min_dist)
           return min_speed - ((min_dist - distance) * speed_per_dist)-movement_constant
        elif distance >= max_dist:
            return 270
           # speed_per_dist = (max_speed - sm_speed) / (max_dist - sm_dist)
           # return max_speed + ((distance - max_dist) * speed_per_dist)-movement_constant
        else:
           (prev_dist, prev_speed) = speeds[0]
           for (cur_dist, cur_speed) in speeds[1:]:
               if distance < cur_dist:
                   speed_per_dist = (cur_speed - prev_speed) / (cur_dist - prev_dist)
                   return prev_speed + ((distance - prev_dist) * speed_per_dist)-movement_constant


rospy.init_node("basketball_logic")
logic = BasketballLogic()

def new_object_callback(message):
    logic.react(message.data.split("\n")[0],
                  message.data.split("\n")[1])


def new_referee_command_callback(message):
    rospy.loginfo("Received referee command: {}".format(message))
    if message.data == "start" and logic.state == State.STOPPED:
        logic.move_to_state(State.FORWARD)
    elif message.data == "stop":
        logic.move_to_state(State.STOPPED)


if __name__ == "__main__":
    rospy.Subscriber("image_processing/objects", String, new_object_callback)
    rospy.Subscriber("robot_main/referee", String, new_referee_command_callback)

    rospy.spin()
