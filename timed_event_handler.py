#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from threading import Lock

from support.singleton import Singleton

class TimedEventHandler:
  __metaclass__ = Singleton

  def __init__(self):
    self.__currentSimTime = None
    self.__events = None
    self.__syncLock = Lock()
  
  def updateSimStep(self, newSimTime):
    self.__syncLock.acquire()
    self.__currentSimTime = newSimTime
    self.__syncLock.release()

  def subscribe(self, name, updateMethod):
    raise NotImplementedError("implement subscribe")

  def unsubscribe(self, name):
    raise NotImplementedError("implement unsubscribe")

  def notify(self):
    raise NotImplementedError("implement notify")
