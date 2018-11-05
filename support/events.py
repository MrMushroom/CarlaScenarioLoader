#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

class Event:
  def __init__(self):
    self.__action = None

  def getAction(self):
    raise NotImplementedError("implement getAction")

class SimTimeEvent(Event):
  def __init__(self):
    Event.__init__(self)

    self.__time = None

  def getEventTime(self):
    raise NotImplementedError("implement getEventTime")

class StateEvent(Event):
  def __init__(self):
    Event.__init__(self)

class EntityEvent(Event):
  def __init__(self):
    Event.__init__(self)

    self.__actors = []

  def getActors(self):
    raise NotImplementedError("implement getActors")