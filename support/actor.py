#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import carla
import math
import random
import sys
import threading
import time

from .control import InputController
from .observer import IObserver
from .present import MondeoPlayerAgentHandler
from .util import Pose, TimeStamp
from timed_event_handler import TimedEventHandler


class Actor(IObserver, threading.Thread):
    def __init__(self, actorType, name, events, enableLogging, pose, speed, timestamp):
        threading.Thread.__init__(self)

        self.name = name

        self._actorType = actorType
        self._name = name
        self._events = events
        self._isLogging = enableLogging
        self._isConnected = False
        self._isRunning = False
        self._currentPose = None
        self._currentSpeed = None
        self._currentTimeStamp = None
        self._desiredPose = pose
        self._desiredSpeed = speed
        self._desiredTimeStamp = timestamp
        self._dataExchangeLock = threading.Lock()
        self._wakeUp = threading.Event()
        self._timeOut = 10.0
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
        if self._isConnected == False:
            print("[Error][CarlaActor::startActing]", self._name, "not connected, cannot start acting")
            return
        if self._isRunning == True:
            print("[Warning][CarlaActor::startActing]", self._name, "already running")
            return

        self._isRunning = True
        threading.Thread.start(self)

    def stopActing(self):
        self._isRunning = False
        self.join()

    # threading.Thread mehtods
    def start(self):
        print("[INFO][CarlaActor::start] Don't use this")
        self.startActing()

    def run(self):
        self._actorThread()

    def update(self, event):
        raise NotImplementedError("implement update")

    @abc.abstractmethod
    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timeout):
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
        self.__inputController = None

    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timeout):
        try:
            self._client = carla.Client(ipAddress, port)
            self._client.set_timeout(timeout)
            print("# try spawning actor", self._name)
            blueprint = self._client.get_world().get_blueprint_library().find("vehicle.lincoln.mkz2017")
            transform = carla.Transform(
                carla.Location(x=272, y=129.5, z=40.0),
                carla.Rotation(yaw=math.pi))
            self.__carlaActor = self._client.get_world().spawn_actor(blueprint, transform)
            if self.__carlaActor is None:
                print("TODO-FIX DEBUG: Couldn't spawn actor")
                # raise RuntimeError("Couldn't spawn actor")

            TimedEventHandler().subscribe(self._name, self.update)
            self._isConnected = True
        except:
            print("[Error][CarlaActor::connectToSimulatorAndEvenHandler] Unexpected error:", sys.exc_info())
            self._isConnected = False
        finally:
            return self._isConnected

    def disconnectFromSimulatorAndEventHandler(self):
        try:
            print("# destroying actor", self._name)
            self._isConnected = False
            TimedEventHandler().unsubscribe(self._name)
            self.__carlaActor.destroy()
            self.__carlaActor = None
            return True
        except:
            print("[Error][CarlaActor::disconnectFromSimulatorAndEventHandler] Unexpected error:", sys.exc_info())
            self._isConnected = False
            return False

    def handleEgo(self):
        # send data to ROS
        print("TODO: handle ego send data to ROS")
        MondeoPlayerAgentHandler().process(self.__carlaActor)
        print(self.__carlaActor.get_transform())

        # receive data from ROS
        if self.__inputController is None:
            self.__inputController = InputController()
        cur_control = self.__inputController.get_cur_control()

        # send data to carla
        print(cur_control)
        cur_control["throttle"] = 0.8
        cur_control["steer"] = 0.0
        cur_control["brake"] = 0.0
        cur_control["hand_brake"] = False
        cur_control["reverse"] = False
        print(cur_control)
        carla_vehicle_control = carla.VehicleControl(cur_control["throttle"],
                                                     cur_control["steer"],
                                                     cur_control["brake"],
                                                     cur_control["hand_brake"],
                                                     cur_control["reverse"])
        self.__carlaActor.apply_control(carla_vehicle_control)

        # transform = carla.Transform(
        #     carla.Location(x=12813, y=19023, z=3900.0),
        #     carla.Rotation(yaw=0.0))
        # self.__carlaActor.set_transform(transform)

    def handleNoneEgo(self):
        # do magic pose stuff
        self._desiredPose = self._currentPose
        self._desiredSpeed = self._desiredSpeed

        # send data to Carla
        transform = carla.Transform(carla.Location(self._desiredPose.getPosition()[0],
                                                   self._desiredPose.getPosition()[1],
                                                   self._desiredPose.getPosition()[2]),
                                    carla.Rotation(self._desiredPose.getOrientation()[1],
                                                   self._desiredPose.getOrientation()[2],
                                                   self._desiredPose.getOrientation()[0]))

        self.__carlaActor.set_transform(transform)

    def update(self, event):
        if event is None:
            pass  # just a normal tick
        else:
            raise NotImplementedError("implement update")

        self._wakeUp.set()

    def _actorThread(self):
        print (self._name, "started acting")
        while(self._isRunning):
            self._dataExchangeLock.acquire()

            try:
                if self._isConnected == False:
                    raise RuntimeError("Actor is not connected to the simulator")

                # receive Carla data
                transform = self.__carlaActor.get_transform()
                self._currentPose = Pose(transform.location.x,
                                         transform.location.y,
                                         transform.location.z,
                                         transform.rotation.roll,
                                         transform.rotation.pitch,
                                         transform.rotation.yaw)
                velocity = self.__carlaActor.get_velocity()
                self._currentSpeed = math.sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0) + pow(velocity.z, 2.0))
                self._currentTimeStamp = TimedEventHandler().getCurrentSimTimeStamp()
                print(self._name, self._currentTimeStamp.getFloat(), self._currentPose, self._currentSpeed)

                # handle data, send to carla - trigger magic stuff
                if(self._name == "Ego"):
                    self.handleEgo()
                else:
                    self.handleNonEgo()

            except:
                print("[Error][CarlaActor::_actorThread] Unexpected error:", sys.exc_info())
                self._isRunning = False

            self._dataExchangeLock.release()

            if not self._wakeUp.wait(self._timeOut):
                print("[Error][CarlaActor::_actorThread] WakeUp Timeout")
                self._isRunning = False

            self._wakeUp.clear()

        print (self._name, "stopped acting")
