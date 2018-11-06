#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


class Action:
    pass


class EntityCondition:
    pass


class SceneDescription:
    pass


class TimeStamp:
    def __init__(self, sec=0, usec=0):
        self.__sec = sec
        self.__usec = usec

    def __eq__(self, other):
        if isinstance(other, TimeStamp):
            return self.__sec == other.__sec and self.__usec == other.__usec
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

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
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.__x = x
        self.__y = y
        self.__z = z
        self.__roll = roll
        self.__pitch = pitch
        self.__yaw = yaw

    def __eq__(self, other):
        if isinstance(other, Pose):
            return self.__x == other.__x and self.__y == other.__y and self.__z == other.__z and self.__roll == other.__roll and self.__pitch == other.__pitch and self.__yaw == other.__yaw
        return False

    def __ne__(self, other):
        return not self.__eq__(other)
