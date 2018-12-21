#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


class Action:
    def __init__(self, pose=None, timestamp=None):
        self.pose = pose
        self.timestamp = timestamp
        self.tags = []

        self.semanticTags = {
            0: None,
            1: "longitudinal",

            "None": 0,
            "longitudinal": 1
        }

        self.longitudinal_speed = None
        self.longitudinal_dynamics_rate = None
        self.longitudinal_dynamics_shape = None


class EntityCondition:
    pass


class SceneDescription:
    pass


class TimeStamp:
    def __init__(self, sec=0, usec=0):
        self.__sec = sec
        self.__usec = usec

    def __lt__(self, other):
        if isinstance(other, TimeStamp):
            return (self.__sec < other.__sec) or (self.__sec == other.__sec and self.__usec < other.__usec)
        return False

    def __le__(self, other):
        if isinstance(other, TimeStamp):
            return (self.__sec < other.__sec) or (self.__sec == other.__sec and self.__usec <= other.__usec)
        return False

    def __gt__(self, other):
        if isinstance(other, TimeStamp):
            return (self.__sec > other.__sec) or (self.__sec == other.__sec and self.__usec > other.__usec)
        return False

    def __ge__(self, other):
        if isinstance(other, TimeStamp):
            return (self.__sec > other.__sec) or (self.__sec == other.__sec and self.__usec >= other.__usec)
        return False

    def __eq__(self, other):
        if isinstance(other, TimeStamp):
            return self.__sec == other.__sec and self.__usec == other.__usec
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def addFloat(self, time):
        self.__sec += int(time)
        self.__usec += (time - int(time))*1000000

    def getInt(self):
        return (self.__sec, self.__usec)

    def getFloat(self):
        time = 0.0
        time += self.__usec
        time /= 1000000.0
        time += self.__sec
        return time

    def setTime(self, sec, usec):
        self.__sec = sec
        self.__usec = usec


class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.__x = x
        self.__y = y
        self.__z = z
        self.__roll = roll
        self.__pitch = pitch
        self.__yaw = yaw

    def getPosition(self):
        return (self.__x, self.__y, self.__z)

    def getOrientation(self):
        return (self.__roll, self.__pitch, self.__yaw)

    def __eq__(self, other):
        if isinstance(other, Pose):
            return self.__x == other.__x and self.__y == other.__y and self.__z == other.__z and self.__roll == other.__roll and self.__pitch == other.__pitch and self.__yaw == other.__yaw
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return str(self.__x) + ", " + str(self.__y) + ", " + str(self.__z) + " " + str(self.__roll) + ", " + str(self.__pitch) + ", " + str(self.__yaw)
