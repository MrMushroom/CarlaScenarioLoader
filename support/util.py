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
            10: "Trajectory",

            "None": 0,
            "longitudinal": 1,
            "Trajectory": 10
        }

        self.longitudinal_speed = None
        self.longitudinal_dynamics_rate = None
        self.longitudinal_dynamics_shape = None

        self.trajectory_lateral_purpose = None  # position/steering
        self.trajectory_longitudinal_none = None  # True/False
        self.trajectory_longitudinal_timing_offset = None
        self.trajectory_longitudinal_timing_scale = None
        self.trajectory_longitudinal_timing_domain_absolute = None
        self.trajectory_longitudinal_timing_domain_relative = None
        self.trajectory_vertex = []
        self.trajectory_vertex_domain = None  # time/distance


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


class Vertex:
    def __init__(self, reference):
        self.reference = reference
        self.pose = None
        self.shape = None
        self.positioning = None
        self.orientation = None
        self.relativeObject = None

        self.shapeTags = {
            0: None,
            1: "Clothoid",
            2: "Polyline",

            "None": 0,
            "Clothoid": 1,
            "Polyline": 2
        }

        self.positioningTags = {
            0: None,
            1: "relative",
            2: "absolute",

            "None": 0,
            "relative": 1,
            "absolute": 2
        }

        self.clothoid_curvature = None
        self.clothoid_curvatureDot = None
        self.clothoid_length = None

    def __lt__(self, other):
        if isinstance(other, Vertex):
            return self.reference < other.reference
        return False

    def __le__(self, other):
        if isinstance(other, Vertex):
            return self.reference <= other.reference
        return False

    def __gt__(self, other):
        if isinstance(other, Vertex):
            return self.reference > other.reference
        return False

    def __ge__(self, other):
        if isinstance(other, Vertex):
            return self.reference >= other.reference
        return False

    def __eq__(self, other):
        if isinstance(other, Vertex):
            return self.reference == other.reference
        return False

    def __ne__(self, other):
        return not self.__eq__(other)
