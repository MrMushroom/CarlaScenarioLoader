#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc

from threading import Lock

from observer import IObserver

class Actor(IObserver):  
  def __init__(self, actorType, name, events, enableLogging, timestamp, pose, speed):
    self.__actorType = actorType
    self.__name = name
    self.__events = events
    self.__isLogging = enableLogging
    self.__isConnected = False
    self.__isRunning = False
    self.__currentPose = None
    self.__currentSpeed = 0.0
    self.__currentTimeStamp = timestamp
    self.__desiredPose = pose
    self.__desiredSpeed = speed
    self.__dataExchangeLock = Lock()
    self.__timedEventHandler = None

  def getIsConnected(self):
    self.__dataExchangeLock.acquire()
    isConnected = self.__isConnected
    self.__dataExchangeLock.release()
    return isConnected

  def getIsRunning(self):
    self.__dataExchangeLock.acquire()
    isRunning = self.__isRunning
    self.__dataExchangeLock.release()
    return isRunning

  def startActing(self):
    raise NotImplementedError("implement startActing")

  def stopActing(self):
    raise NotImplementedError("implement stopActing")

  def update(self, event):
    raise NotImplementedError("implement update")
  
  @abc.abstractmethod
  def connectToSimulatorAndEvenHandler(self, ipAddress, port, timedEventHandler):
    pass

  @abc.abstractmethod
  def disconnectFromSimulatorAndEventHandler(self):
    pass

  @abc.abstractmethod
  def actorThread(self):
    pass


class CarlaActor(Actor):
  def __init__(self, actorType, name, events, enableLogging, timestamp, pose, speed):
    Actor.__init__(self, actorType, name, events, enableLogging, timestamp, pose, speed)
    
  def connectToSimulatorAndEvenHandler(self, ipAddress, port, timedEventHandler):
    raise NotImplementedError("implement connectToSimulatorAndEvenHandler")

  def disconnectFromSimulatorAndEventHandler(self):
    raise NotImplementedError("implement disconnectFromSimulatorAndEventHandler")

  def actorThread(self):
    raise NotImplementedError("implement actorThread")