
#!/usr/bin/env python
# Taken or adapted from
# https://bitbucket.org/MartinAppo/diploaf-2017/src/master/hardware_module/src/hardware_module/comport_mainboard.py

import serial
import threading
import time
import subprocess
import rospy
from std_msgs.msg import String


class ComportMainboard(threading.Thread):
    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)
        self.buffered = ""

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened and not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=False , write_timeout = 0.5)
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print "mainboard: Port opened successfully"
            except Exception as e:
                print(e)
		
                continue

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None:
            try:
                rospy.loginfo("Sending " + comm)
                self.connection.write(comm + "\n")
            except:
                print('mainboard: err write ' + comm)

    def read(self):
	s = ""
	t = ""
	while not rospy.is_shutdown():
	    if self.connection is None or  not self.connection.in_waiting:
		break
	    t= self.connection.read()
	    #print t+ " tere"
	    if t == "\n":
	        break
	    s+=t
        return s

    def toggle_red_led(self):
        if self.connection is not None and self.connection_opened:
            self.write("r")

    def set_wheels(self, wheel_one, wheel_two, wheel_three):
        if self.connection_opened:
            self.write("sd:{}:{}:{}".format(wheel_one, wheel_two, wheel_three))

    def set_throw(self, speed):
        self.write("d:{}".format(speed))

    def set_servo(self, value):
        self.write("sv:{}".format(value))

    def send_ping(self, robotID, fieldID):
        ack_message = 'a' + fieldID + robotID + 'ACK-----'
        self.write("rf:{}".format(ack_message))

    def close(self):
        if self.connection is not None and self.connection.isOpen():  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return
