#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from threading import Lock

from support.singleton import Singleton
from support.util import TimeStamp


class TimedEventHandler(metaclass=Singleton):
    def __init__(self):
        self.__currentSimTime = None
        self.__events = None
        self.__subscribers = {}
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

    def updateSimStep(self, newSimTime):
        self.__syncLock.acquire()
        self.__currentSimTime = newSimTime.platform_timestamp
        self.__notify()
        self.__syncLock.release()

    def subscribe(self, name, updateMethod):
        self.__syncLock.acquire()
        if name in self.__subscribers:
            raise Exception(name, "already subscribed")
        else:
            self.__subscribers[name] = updateMethod
        self.__syncLock.release()

    def unsubscribe(self, name):
        self.__syncLock.acquire()
        del self.__subscribers[name]
        self.__syncLock.release()

    def __notify(self):
        for subscriber, method in self.__subscribers.items():
            # check events for subscriber
            event = None
            method(event)
