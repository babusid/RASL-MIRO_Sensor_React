#############
# Allows for simple writing of code to be used with Consequential robotics MIRO robot
# Created by Sidharth Babu  7/12/2018
# default behavior is to move around whilst avoiding any obstacles and reacting to touch

import math
import time
import rospy

from interfaces import *


class SecondaryInterface:

    def __init__(self, robotname, linear, angular):
        self.default_linear = linear
        self.default_angular = angular
        self.pint = primary_interface(robotname)

    def defaultmovestate(self):
        # utilizes the sonar sensors so that
        # the robot can move around without hitting things
        self.pint.ear_rotate = [1, 0]  # move one ear
        self.pint.head_move(0, .2)  # turn to left side
        time.sleep(.2)
        x = self.pint.sonar_range  # left side turn range value
        time.sleep(.2)
        self.pint.head_move()  # set to middle
        self.pint.ear_rotate = [0, 0]  # set ears to normal
        time.sleep(.2)
        self.pint.head_move(0, -.2)  # turn to right side
        self.pint.ear_rotate = [0, 1]  # move other ear
        time.sleep(.2)
        y = self.pint.sonar_range  # right side turn range value
        time.sleep(.2)
        z = (x + y) / 2  # average distance from both sides
        self.pint.head_move()  # set back to middle
        print(str(x) + '| leftval')
        print(str(y) + '| rightval')
        if x > 0 and y > 0:  # make sure its registering some value

            if z < .3:  # its too close; get away
                print('straight back')
                self.pint.tail_move(-1)
                self.pint.drive_straight(-.2)
                time.sleep(2)
                self.pint.turn(math.pi)
                time.sleep(1)
                self.pint.stop_moving()
                self.pint.tail_move(0)

            else:  # nothing too close

                if x > y:  # right side is closer than left; move left
                    print('left')
                    self.pint.turn(math.pi)
                    time.sleep(.25)
                    self.pint.drive_straight()
                    time.sleep(1)
                    self.pint.stop_moving()

                elif y > x:  # left side is closer than right; move right
                    print('right')
                    self.pint.turn(-math.pi)
                    time.sleep(.25)
                    self.pint.drive_straight()
                    time.sleep(1)
                    self.pint.stop_moving()

                elif y == x:  # both sides are equidistant
                    print('straight forward')
                    self.pint.drive_straight(.2)
                    time.sleep(.5)
                    self.pint.stop_moving()

        else:  # if nothing registers, move until something does
            self.pint.drive_straight(.2)
            time.sleep(.5)
            self.pint.stop_moving()

    def sensorinterrupt(self):
        while not rospy.is_shutdown():
            # create your if statement based sensor routine here
            if 1 in self.pint.touch_body:
                self.pint.stop_moving()
                self.pint.tail_move()
                self.pint.head_move(1)
                time.sleep(.5)
                self.pint.head_move()

            elif 1 in self.pint.touch_head:
                self.pint.head_nod_sideways()

            # below returns robot to default state
            else:
                self.defaultmovestate()
