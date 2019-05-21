#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import carla
import datetime
import prctl
import sys
import threading
import time

from support.present import ClockHandler

from timed_event_handler import TimedEventHandler


class SimulatorControl():
    def __init__(self, simName):
        self._simName = simName
        self._isConnected = False
        self._isRunning = False
        self._stateEvents = None
        self._statusLock = threading.Lock()

    def getIsConnected(self):
        self._statusLock.acquire()
        isConnected = self._isConnected
        self._statusLock.release()
        return isConnected

    def getIsRunning(self):
        self._statusLock.acquire()
        isRunning = self._isRunning
        self._statusLock.release()
        return isRunning

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
    def __init__(self, simulatorIP, simulatorPort, simulatorTimeout):
        SimulatorControl.__init__(self, "Carla")
        self._simIP = simulatorIP
        self._simPort = simulatorPort
        self._simTimeout = simulatorTimeout
        self._client = None

    def connect(self):
        try:
            prctl.set_name("carlaSimCtl")
            self._client = carla.Client(self._simIP, self._simPort)
            prctl.set_name("burn")
            self._client.set_timeout(self._simTimeout)
            print("[INFO] Connecting", self._client.get_client_version(), "to", self._client.get_server_version())
            settings = self._client.get_world().get_settings()
            settings.synchronous_mode = True
            self._client.get_world().apply_settings(settings)
            print("[INFO] Applied Settings: %r", settings)
            self._isConnected = True
            self.run()
            return True
        except:
            print("[Error][CarlaSimulatorControl::connect] Unexpected error:", sys.exc_info())
            self._isConnected = False
            return False

    def disconnect(self):
        # TODO BUG Deadlock. At program end, run_cb has lock somewhere
        self._statusLock.acquire()
        self._isRunning = False
        self._isConnected = False

        #switch back syncmode settings
        settings = self._client.get_world().get_settings()
        settings.synchronous_mode = False
        self._client.get_world().apply_settings(settings)

        self._client = None
        self._statusLock.release()

    def loadScene(self, sceneDescription):
        print("[ERROR][CarlaSimulatorControl::loadScene] loadScene not implemented into Carla 0.9.0")

    def run(self):
        self._isRunning = True
        self._client.get_world().on_tick(self.run_cb)

    def run_cb(self, timestamp):
        self._statusLock.acquire()
        isRunning = self._isRunning
        self._statusLock.release()

        # TODO BUG Deadlock. if wont prevent deadlocking at program end with disconnect
        if isRunning:
            TimedEventHandler().updateSimStep(timestamp)
            ClockHandler().process()
        else:
            pass

        try:
            TimedEventHandler().syncBarrier()
        except threading.BrokenBarrierError:
            return  # will happen at program end

        self._client.get_world().tick()
