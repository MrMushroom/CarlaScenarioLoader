#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import carla
import sys

from threading import Lock

from .observer import IObserver
from .util import Pose, TimeStamp


class Actor(IObserver):
    def __init__(self, actorType, name, events, enableLogging, pose, speed, timestamp):
        self._actorType = actorType
        self._name = name
        self._events = events
        self._isLogging = enableLogging
        self._isConnected = False
        self._isRunning = False
        self._currentPose = pose
        self._currentSpeed = speed
        self._currentTimeStamp = timestamp
        self._desiredPose = None
        self._desiredSpeed = None
        self._desiredTimeStamp = timestamp
        self._dataExchangeLock = Lock()
        self._timedEventHandler = None
        self._client = None

    def getName(self):
        return self._name

    def getIsConnected(self):
        self._dataExchangeLock.acquire()
        isConnected = self._isConnected
        self._dataExchangeLock.release()
        return isConnected

    def getIsRunning(self):
        self._dataExchangeLock.acquire()
        isRunning = self._isRunning
        self._dataExchangeLock.release()
        return isRunning

    def setInit(self, speed, pose):
        self._desiredSpeed = speed
        self._desiredPose = pose
        self._desiredTimeStamp = TimeStamp()

    def startActing(self):
        raise NotImplementedError("implement startActing")

    def stopActing(self):
        raise NotImplementedError("implement stopActing")

    # def update(self, event):
    #     raise NotImplementedError("implement update")

    @abc.abstractmethod
    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timeout, timedEventHandler):
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
        self.__carlaActor = None

    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timeout, timedEventHandler):
        try:
            self._client = carla.Client(ipAddress, port)
            self._client.set_timeout(timeout)
            print("# try spawning actor", self._name)
            blueprint = self._client.get_world().get_blueprint_library().find("vehicle.tesla.model3")
            transform = carla.Transform(
                carla.Location(x=140.0, y=199.0, z=40.0),
                carla.Rotation(yaw=0.0))
            self.__carlaActor = self._client.get_world().try_spawn_actor(blueprint, transform)
            if self.__carlaActor is None:
                raise Exception("Couldn't spawn actor")

            print("TODO connect to timed event handler")
            return True
        except:
            print("[Error][CarlaActor::connectToSimulatorAndEvenHandler] Unexpected error:", sys.exc_info())
            return False

    def disconnectFromSimulatorAndEventHandler(self):
        try:
            print("# destroying actor", self._name)
            self.__carlaActor.destroy()
            print("TODO disconnect TCP connection maybe?")
            self.__carlaActor = None
            print("TODO disconnect from timed event handler")
            return True
        except:
            print("[Error][CarlaActor::disconnectFromSimulatorAndEventHandler] Unexpected error:", sys.exc_info())
            return False

    def _actorThread(self):
        print (self._name, "started acting")
        while(self._isRunning):
            # receive Carla data
            # send Carla data to ROS

            # receive ROS data
            # send ROS data to Carla
            continue
        print (self._name, "stopped acting")
