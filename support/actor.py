#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc

from threading import Lock

from .observer import IObserver
from .util import Pose, TimeStamp


class Actor(IObserver):
    def __init__(self, actorType, name, events, enableLogging, pose, speed, timestamp):
        self.__actorType = actorType
        self.__name = name
        self.__events = events
        self.__isLogging = enableLogging
        self.__isConnected = False
        self.__isRunning = False
        self.__currentPose = pose
        self.__currentSpeed = speed
        self.__currentTimeStamp = timestamp
        self.__desiredPose = None
        self.__desiredSpeed = None
        self.__dataExchangeLock = Lock()
        self.__timedEventHandler = None

    def getName(self):
        return self.__name

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

    def setAction(self, action):
        print (action)
        raise NotImplementedError("implement setAction")

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
    def _actorThread(self):
        pass


class CarlaActor(Actor):
    def __init__(self, actorType, name, events=[], enableLogging=False, pose=None, speed=None, timestamp=None):
        Actor.__init__(self, actorType, name, events, enableLogging, pose, speed, timestamp)

    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timedEventHandler):
        raise NotImplementedError("implement connectToSimulatorAndEvenHandler")

    def disconnectFromSimulatorAndEventHandler(self):
        raise NotImplementedError("implement disconnectFromSimulatorAndEventHandler")

    def _actorThread(self):
        raise NotImplementedError("implement actorThread")
