#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import carla
import datetime
import sys
import time

from support.singleton import Singleton


class SimulatorControl:
    __metaclass__ = Singleton

    def __init__(self, simName):
        self.__simName = simName
        self.__isConnected = False
        self.__isRunning = False
        self.__stateEvents = None

    def getIsConnected(self):
        raise NotImplementedError("implement getIsConnected")

    def getIsRunning(self):
        raise NotImplementedError("implement getIsRunning")

    @abc.abstractmethod
    def connect(self):
        pass

    @abc.abstractmethod
    def disconnect(self):
        pass

    @abc.abstractmethod
    def loadScene(self, sceneDescription):
        pass

    @abc.abstractmethod
    def run(self):
        pass


class CarlaSimulatorControl(SimulatorControl):
    def __init__(self, simulatorIP):
        SimulatorControl.__init__(self, "Carla")
        self._simIP = simulatorIP
        self._simPort = 2000
        self._client = None
        self._timeout = 2.0

    def connect(self):
        try:
            self._client = carla.Client(self._simIP, self._simPort)
            self._client.set_timeout(self._timeout)
            self.run()
            return True
        except:
            print("[Error][CarlaSimulatorControl::connect] Unexpected error:", sys.exc_info())
            return False

    def disconnect(self):
        print("[WARNING][CarlaSimulatorControl::disconnect] disconnect not yet fully implemented into Carla 0.9.0")

    def loadScene(self, sceneDescription):
        print("[ERROR][CarlaSimulatorControl::loadScene] loadScene not implemented into Carla 0.9.0")

    def run(self):
        self._client.get_world().on_tick(self.run_cb)

    def run_cb(self, timestamp):
        print(timestamp)
