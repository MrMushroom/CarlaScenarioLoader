#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from .util import Action


class Event:
    def __init__(self, action, startCondition=None):
        self.__action = action
        self.__startCondition = startCondition

    def getAction(self):
        return self.__action

    def getStartCondition(self):
        return self.__startCondition


class SimTimeEvent(Event):
    def __init__(self, action, startCondition):
        Event.__init__(self, action)

        self.__time = action.timestamp

    def getEventTime(self):
        self.__time


class StateEvent(Event):
    def __init__(self, action, startCondition):
        Event.__init__(self, action)


class EntityEvent(Event):
    def __init__(self, action, actors, startCondition):
        Event.__init__(self, action)

        self.__actors = actors

    def getActors(self):
        return self.__actors


class StartCondition:
    def __init__(self):
        self.priority = None

        # timing
        self.delay = None
        self.edge = None

        # ByEntity
        self.triggeringEntity = None

        # ReachPosition
        self.pose_tolerance = None
        self.pose = None
