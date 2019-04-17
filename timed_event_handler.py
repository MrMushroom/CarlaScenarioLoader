#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from threading import Barrier, Lock

from support.singleton import Singleton
from support.util import TimeStamp


class TimedEventHandler(metaclass=Singleton):
    def __init__(self):
        self.__currentSimTime = None
        self.__previousSimTime = None
        self.__events = None
        self.__subscribers = {}
        self.__syncBarrier = Barrier(1) # (1): Simulator Control Blocks too
        self.__syncLock = Lock()
        self.__cleared = True

    def clear(self):
        self.__init__()

    def getCurrentSimTime(self):
        self.__syncLock.acquire()
        simtime = self.__currentSimTime
        self.__syncLock.release()
        return simtime

    def getCurrentSimTimeStamp(self):
        self.__syncLock.acquire()
        simtime = self.__currentSimTime
        self.__syncLock.release()
        timestamp = TimeStamp(int(simtime), (simtime - int(simtime))*1000000)
        return timestamp

    def getPreviousSimTimeStamp(self):
        self.__syncLock.acquire()
        simtime = self.__previousSimTime
        self.__syncLock.release()
        timestamp = TimeStamp(int(simtime), (simtime - int(simtime))*1000000)
        return timestamp

    def getSimTimeDiff(self):
        self.__syncLock.acquire()
        if self.__previousSimTime is None:
            timeDiff = None
        else:
            timeDiff = self.__currentSimTime - self.__previousSimTime
        self.__syncLock.release()
        return timeDiff

    def updateSimStep(self, newSimTime):
        self.__syncLock.acquire()
        self.__previousSimTime = self.__currentSimTime
        self.__currentSimTime = newSimTime.platform_timestamp
        self.__notify()
        self.__syncLock.release()

    def stop(self):
        self.__syncLock.acquire()
        self.__syncBarrier.abort()
        self.__notify()
        self.__syncLock.release()

    def subscribe(self, name, updateMethod):
        self.__syncLock.acquire()
        if self.__syncBarrier.n_waiting != 0:
            raise Exception(name, "tried to subscribe during runtime (syncBarrier has", self.__syncBarrier.n_waiting, "threads waiting)")
        if name in self.__subscribers:
            raise Exception(name, "already subscribed")
        else:
            self.__syncBarrier = Barrier(self.__syncBarrier.parties + 1)
            self.__subscribers[name] = updateMethod
        self.__syncLock.release()

    def syncBarrier(self):
        self.__syncBarrier.wait(timeout=1.0)

    def unsubscribe(self, name):
        self.__syncLock.acquire()
        if self.__syncBarrier.n_waiting != 0:
            raise Exception(name, "tried to unsubscribe during runtime (syncBarrier has", self.__syncBarrier.n_waiting, "threads waiting)")
        del self.__subscribers[name]
        self.__syncBarrier = Barrier(self.__syncBarrier.parties - 1)
        self.__syncLock.release()

    def __notify(self):
        for subscriber, method in self.__subscribers.items():
            # check events for subscriber
            event = None
            method(event)
