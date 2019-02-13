#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
# Inspired by carla_ros_bridge (Carla 0.8.4)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Class to handle control input
"""

import carla
import numpy
import rospy
import threading

from dbw_mkz_msgs.msg import BrakeCmd, GearCmd, SteeringCmd, ThrottleCmd
from common_msgs.msg import Path

from .singleton import Singleton


class InputController(object, metaclass=Singleton):
    """
    Class to handle ros input command
    """

    def __init__(self):
        # current control command
        self.cur_control = {
            'steer': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'hand_brake': False,
            'reverse': False,
            'gear': 0
        }
        self.old_control = self.cur_control

        # gear dictionary for GearCmd
        self.gears = {
            0: "None",
            1: "P",
            2: "R",
            3: "N",
            4: "D",
            5: "L",

            "None": 0,
            "P": 1,
            "R": 2,
            "N": 3,
            "D": 4,
            "L": 5
        }

        # ford mondeo steering ratio 14.8
        # max_steer angle according to SteerCmd = 8.2
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.2)

        self.lock_cur_control = threading.Lock()

        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd,
                         self.recv_brake_cmd, queue_size=1)
        rospy.Subscriber('/vehicle/gear_cmd', GearCmd,
                         self.recv_gear_cmd, queue_size=1)
        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd,
                         self.recv_steering_cmd, queue_size=1)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd,
                         self.recv_throttle_cmd, queue_size=1)

        # Backup functionality
        self.local_path = Path()
        self.lock_local_path = threading.Lock()
        rospy.Subscriber('/local_path', Path,
                         self.recv_local_path, queue_size=1)

    def get_cur_control(self):
        self.lock_cur_control.acquire()

        control = self.cur_control
        self.old_control = self.cur_control

        self.lock_cur_control.release()

        return control

    def get_old_control(self):
        self.lock_cur_control.acquire()

        control = self.old_control

        self.lock_cur_control.release()

        return control

    def recv_brake_cmd(self, msg):
        self.lock_cur_control.acquire()

        self.cur_control['brake'] = msg.pedal_cmd

        self.lock_cur_control.release()

    def recv_gear_cmd(self, msg):
        self.lock_cur_control.acquire()

        self.cur_control['gear'] = msg.cmd.gear

        self.lock_cur_control.release()

    def recv_steering_cmd(self, msg):
        self.lock_cur_control.acquire()

        steering_angle = msg.steering_wheel_angle_cmd * self.steer_ratio
        max_steering_angle = self.max_steer_angle * self.steer_ratio
        steer = steering_angle/max_steering_angle
        steer = numpy.clip(steer, -1.0, 1.0)

        self.cur_control['steer'] = -steer

        self.lock_cur_control.release()

    def recv_throttle_cmd(self, msg):
        self.lock_cur_control.acquire()

        self.cur_control['hand_brake'] = False
        self.cur_control['reverse'] = False
        self.cur_control['throttle'] = 0.0

        if self.gears[self.cur_control['gear']] == "None":
            pass
        elif self.gears[self.cur_control['gear']] == "P":
            self.cur_control['hand_brake'] = True
            pass
        elif self.gears[self.cur_control['gear']] == "R":
            self.cur_control['reverse'] = True
            self.cur_control['throttle'] = msg.pedal_cmd    # 0..1
            pass
        elif self.gears[self.cur_control['gear']] == "N":
            pass
        elif self.gears[self.cur_control['gear']] == "D":
            self.cur_control['throttle'] = msg.pedal_cmd    # 0..1
            pass
        elif self.gears[self.cur_control['gear']] == "L":
            self.cur_control['throttle'] = msg.pedal_cmd    # 0..1
            pass

        self.lock_cur_control.release()

    def get_local_path(self):
        self.lock_local_path.acquire()

        local_path = self.local_path

        self.lock_local_path.release()

        return local_path
    
    def recv_local_path(self, msg):
        self.lock_local_path.acquire()
        self.local_path = msg
        self.lock_local_path.release()