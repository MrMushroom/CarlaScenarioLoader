#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import carla
import datetime
import math
import prctl
import random
import sys
import threading
import time

from collections import deque

from . import maneuvers
from .control import InputController
from .observer import IObserver
from .present import MondeoPlayerAgentHandler
from .util import Action, Pose, TimeStamp
from timed_event_handler import TimedEventHandler


class Actor(IObserver, threading.Thread):
    def __init__(self, actorType, name, enableLogging, pose, speed, timestamp):
        threading.Thread.__init__(self, name=name)
        self._actorType = actorType
        self._name = name
        self._events = deque()
        self._isLogging = enableLogging
        self._isConnected = False
        self._isRunning = False
        self._action = None
        self._currentPose = None
        self._currentSpeed = None
        self._currentTimeStamp = None
        self._executionQueue = deque()
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

    def setAction(self, action):
        self._dataExchangeLock.acquire()
        self._setAction(action)
        self._dataExchangeLock.release()

    def _setAction(self, action):
        self._action = action
        self._executionQueue.clear()

    def startActing(self):
        if self._isConnected == False:
            print("[Error][Actor::startActing]", self._name, "not connected, cannot start acting")
            return
        if self._isRunning == True:
            print("[Warning][Actor::startActing]", self._name, "already running")
            return

        self._isRunning = True
        threading.Thread.start(self)

    def stopActing(self):
        self._isRunning = False
        self.join()

    # threading.Thread mehtods
    def start(self):
        print("[INFO][Actor::start] Don't use this")
        self.startActing()

    def run(self):
        prctl.set_name(self._name)
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
    def __init__(self, actorType, name, enableLogging=False, pose=None, speed=None, timestamp=None):
        Actor.__init__(self, actorType, name, enableLogging, pose, speed, timestamp)
        self.__carlaActor = None
        self.__carlaCollisionSensor = None
        self.__inputController = None
        self.__wakeUpOnScenarioEnd = None

        self.collisionEvent = None

    def connectToSimulatorAndEvenHandler(self, ipAddress, port, timeout, wakeUpOnScenarioEnd):
        self.__wakeUpOnScenarioEnd = wakeUpOnScenarioEnd
        
        try:
            prctl.set_name("sim" + self._name)
            self._client = carla.Client(ipAddress, port)
            prctl.set_name("burn")
            self._client.set_timeout(timeout)
            print("# spawning actor", self._name)
            blueprint = self._client.get_world().get_blueprint_library().find("vehicle.lincoln.mkz2017")
            # blueprint = self._client.get_world().get_blueprint_library().find("vehicle.ford.mustang")
            transform = carla.Transform(
                carla.Location(x=self._desiredPose.getPosition()[0],
                               y=self._desiredPose.getPosition()[1],
                               z=self._desiredPose.getPosition()[2]),
                carla.Rotation(roll=math.degrees(self._desiredPose.getOrientation()[0]),
                               pitch=math.degrees(self._desiredPose.getOrientation()[1]),
                               yaw=math.degrees(self._desiredPose.getOrientation()[2])))
            self.__carlaActor = self._client.get_world().spawn_actor(blueprint, transform)
            if self.__carlaActor is None:
                raise RuntimeError("Couldn't spawn actor")

            # add sensors
            try:
                if(self._name == "Ego"):
                    bp = self._client.get_world().get_blueprint_library().find('sensor.other.collision')
                    self.__carlaCollisionSensor = self._client.get_world().spawn_actor(bp, carla.Transform(), attach_to=self.__carlaActor)
                    self.__carlaCollisionSensor.listen(lambda event: self.onCollision(event))
            except Exception as e:
                print(e)

            # wait for vehicle spawn
            if(transform.location.x != 0 or transform.location.y != 0 or transform.location.z != 0 or
                    transform.rotation.roll != 0 or transform.rotation.pitch != 0 or transform.rotation.yaw != 0):
                # desired location is not origin -> wait for vehicle spawn
                transform = self.__carlaActor.get_transform()
                while(transform.location.x == 0 and transform.location.y == 0 and transform.location.z == 0 and
                      transform.rotation.roll == 0 and transform.rotation.pitch == 0 and transform.rotation.yaw == 0):
                    transform = self.__carlaActor.get_transform()

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

            if(self.__carlaCollisionSensor != None):
                self.__carlaCollisionSensor.destroy()
                self.__carlaCollisionSensor = None

            self.__carlaActor.destroy()
            self.__carlaActor = None

            return True
        except:
            print("[Error][CarlaActor::disconnectFromSimulatorAndEventHandler] Unexpected error:", sys.exc_info())
            self._isConnected = False
            return False

    def addEntityEvent(self, event):
        self._dataExchangeLock.acquire()
        self._events.append(event)
        self._dataExchangeLock.release()

    def checkConditionTriggered(self, sc):
        isConditionTriggered = False

        if sc.pose != None:
            distance = math.sqrt(pow(self._currentPose.getPosition()[0]-sc.pose.getPosition()[0], 2.0) +
                                 pow(self._currentPose.getPosition()[1]-sc.pose.getPosition()[1], 2.0) +
                                 pow(self._currentPose.getPosition()[2]-sc.pose.getPosition()[2], 2.0))
            if distance < sc.pose_tolerance:
                isConditionTriggered = True
            else:
                isConditionTriggered = False
        else:
            print("[WARNING][CarlaActor::checkConditionTriggered] Implementation Missing. This should not be reached")

        return isConditionTriggered

    def handleEvents(self):
        # check the startconditions
        for event in self._events:
            sc = event.getStartCondition()
            if sc != None:
                if sc.isConditionTrue:
                    pass
                elif sc.isConditionMetEdge:
                    if TimedEventHandler.getCurrentSimTimeStamp().getFloat() >= sc.timestampConditionTriggered.getFloat() + sc.delay:
                        sc.isConditionTrue = True
                    else:
                        sc.isConditionTrue = False
                elif sc.isConditionTriggered:
                    if sc.edge == "falling" or sc.edge == "any":
                        sc.isConditionMetEdge = not self.checkConditionTriggered(sc)
                    else:
                        print("[WARNING][CarlaActor::handleEvents] Implementation Missing. This should not be reached")
                else:
                    sc.isConditionTriggered = self.checkConditionTriggered(sc)
                    if sc.isConditionTriggered and (sc.edge == "rising" or sc.edge == "any"):
                        sc.isConditionMetEdge = True
                        if sc.delay == 0.0:
                            sc.isConditionTrue = True
                        else:
                            sc.timestampConditionTriggered = TimedEventHandler.getCurrentSimTimeStamp()
                    else:
                        pass  # edge falling

        remainingEvents = deque()
        while(len(self._events) > 0):
            event = self._events.popleft()
            if event.getStartCondition().isConditionTrue:
                if event.getStartCondition().priority == "overwrite":
                    if hasattr(event, "getActors"):
                        for actor in event.getActors():
                            if(actor is self):
                                self._setAction(event.getAction())
                            else:
                                actor.setAction(event.getAction())
                    else:
                        self._setAction(event.getAction())
                else:
                    print("[WARNING][CarlaActor::handleEvents] Implementation Missing. This should not be reached")
            else:
                remainingEvents.append(event)
        self._events = remainingEvents

    def handleExecutionQueue(self):
        # check if queue full
        if len(self._executionQueue) > 1:
            if self._executionQueue[-1].timestamp >= self._currentTimeStamp:
                return len(self._executionQueue)  # full and useable

        # current Pose is always the result of the previous TimeStamp
        if(self._previousTimeStamp is None):
            self._previousTimeStamp = self._currentTimeStamp  # happens at startup

        # queue hast just one item left, start magic execution stuff
        if self._action is None:
            self._executionQueue = maneuvers.constantStraightAhead(self._currentPose, self._previousTimeStamp, self._desiredSpeed)
        else:
            # TODO build a better decision tree for the action
            if(len(self._action.tags) == 1 and self._action.semanticTags["longitudinal"] in self._action.tags):
                # straight ahead
                if(self._action.longitudinal_dynamics_shape == "step"):
                    self._desiredSpeed = self._action.longitudinal_speed
                    self._executionQueue = maneuvers.constantStraightAhead(self._currentPose, self._previousTimeStamp, self._desiredSpeed)
                else:
                    print("[WARNING][CarlaActor::handleExecutionQueue] Implementation Missing for longitudinal action!")
            elif(len(self._action.tags) == 1 and self._action.semanticTags["Trajectory"] in self._action.tags):
                # TODO info
                self._desiredSpeed = self._desiredSpeed
                if (self._action.trajectory_lateral_purpose == "steering"):
                    if(self._action.trajectory_longitudinal_none == False):
                        print("[WARNING][CarlaActor::handleExecutionQueue] Implementation Missing for Trajectory action (longitudinal_none == True)!")
                    if(self._action.trajectory_longitudinal_timing_offset or
                            self._action.trajectory_longitudinal_timing_scale or
                            self._action.trajectory_longitudinal_timing_domain_absolute or
                            self._action.trajectory_longitudinal_timing_domain_relative):
                        print("[WARNING][CarlaActor::handleExecutionQueue] Implementation Missing for Trajectory action (trajectory_timing)!")
                    else:
                        self._executionQueue = maneuvers.trajectory(self._action.trajectory_vertex, self._action.trajectory_vertex_domain, self._currentPose, self._previousTimeStamp, self._desiredSpeed)
                else:
                    print("[WARNING][CarlaActor::handleExecutionQueue] Implementation Missing for Trajectory action (lateral_purpose)!")
            else:
                print("[WARNING][CarlaActor::handleExecutionQueue] Implementation Missing. Unable to handle new action type")

            self._action = None
        return len(self._executionQueue)

    def handleEgo(self):
        # send data to ROS
        MondeoPlayerAgentHandler().process(self.__carlaActor)
        MondeoPlayerAgentHandler().processGodSensor(self.__carlaActor)

        # receive data from ROS
        if self.__inputController is None:
            self.__inputController = InputController()
        cur_control = self.__inputController.get_cur_control()

        # check action
        if self._action is None:
            pass
        else:
            # TODO build a better decision tree for the action
            if(len(self._action.tags) == 1 and self._action.semanticTags["longitudinal"] in self._action.tags):
                # set speed
                if(self._action.longitudinal_dynamics_shape == "step"):
                    self._desiredSpeed = self._action.longitudinal_speed
                else:
                    print("[WARNING][CarlaActor::handleEgo] Implementation Missing for longitudinal action!")
            else:
                print("[WARNING][CarlaActor::handleEgo] Implementation Missing. Unable to handle new action type")

            self._action = None

        # send data to carla
        carla_vehicle_control = carla.VehicleControl(cur_control["throttle"],
                                                     cur_control["steer"],
                                                     cur_control["brake"],
                                                     cur_control["hand_brake"],
                                                     cur_control["reverse"])
        self.__carlaActor.apply_control(carla_vehicle_control)

        # check if end position reached
        if len(self._events) == 0 and self._desiredSpeed == 0.0 and self._currentSpeed == 0.0:
            self.__wakeUpOnScenarioEnd.set()

    def handleNonEgo(self):
        # do magic pose stuff
        self.handleExecutionQueue()

        # TODO check: due to execution stack, there might be an inconsitency (missing pose for timestamp) when stack gets empty
        prevAction = None
        nextAction = None
        while len(self._executionQueue) > 0:
            prevAction = nextAction
            nextAction = self._executionQueue.popleft()
            if(nextAction.timestamp >= self._currentTimeStamp):
                break

        if nextAction is None:
            self._desiredPose = self._currentPose
        elif nextAction.timestamp == self._currentTimeStamp:
            self._desiredPose = nextAction.pose
        elif prevAction is None:
            prevAction = Action(self._currentPose, self._previousTimeStamp)
            self._desiredPose = maneuvers.interpolateActions(prevAction, nextAction, self._currentTimeStamp)
        else:  # (prevAction is not None and nextAction is not None)
            self._desiredPose = maneuvers.interpolateActions(prevAction, nextAction, self._currentTimeStamp)

        print(self._desiredSpeed, self._currentSpeed)
        if nextAction is not None:
            if nextAction.longitudinal_speed is not None:
                self._desiredSpeed = nextAction.longitudinal_speed
                # TODO refactor speed setting to allow shapes!
                print("TODO refactor speed setting to allow shapes!")

        # send data to Carla
        transform = carla.Transform(carla.Location(self._desiredPose.getPosition()[0],
                                                   self._desiredPose.getPosition()[1],
                                                   self._desiredPose.getPosition()[2]),
                                    carla.Rotation(math.degrees(self._desiredPose.getOrientation()[1]),
                                                   math.degrees(self._desiredPose.getOrientation()[2]),
                                                   math.degrees(self._desiredPose.getOrientation()[0])))

        pitch = self._desiredPose.getOrientation()[1]
        yaw = self._desiredPose.getOrientation()[2]
        velocity = self.__carlaActor.get_velocity()
        print("a", velocity)
        print("u", self.__carlaActor.get_location())
        velocity.x = self._desiredSpeed * math.cos(yaw) * math.cos(pitch)
        velocity.y = self._desiredSpeed * math.sin(yaw) * math.cos(pitch)
        #velocity.z = self._desiredSpeed * math.sin(pitch) # dont set, let the vehicle drop
        print("b", velocity)

        # if self._name == "Target1":
        #     print(self._name, TimedEventHandler().getSimTimeDiff(),  transform)

        self.__carlaActor.set_transform(transform)
        #self.__carlaActor.set_velocity(velocity)

    def onCollision(self, event):
        if(self.__wakeUpOnScenarioEnd != None):
            self.collisionEvent = event
            self.__wakeUpOnScenarioEnd.set()

    def update(self, event):
        if event is None:
            pass  # just a normal tick
        else:
            self._dataExchangeLock.acquire()
            self._events.append(event)
            self._dataExchangeLock.release()

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
                                         math.radians(transform.rotation.roll),
                                         math.radians(transform.rotation.pitch),
                                         math.radians(transform.rotation.yaw))
                velocity = self.__carlaActor.get_velocity()
                self._currentSpeed = math.sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0) + pow(velocity.z, 2.0))
                self._currentTimeStamp = TimedEventHandler().getCurrentSimTimeStamp()
                self._previousTimeStamp = TimedEventHandler().getPreviousSimTimeStamp()
                # print(self._name, self._currentTimeStamp.getFloat(), self._currentPose, self._currentSpeed)

                # handleEvents()
                self.handleEvents()

                # handle data, send to carla - trigger magic stuff
                if(self._name == "Ego"):
                    self.handleEgo()
                else:
                    self.handleNonEgo()

            except:
                print("[Error][CarlaActor::_actorThread] Unexpected error:", sys.exc_info())
                self._isRunning = False

            self._dataExchangeLock.release()

            # TODO INFO time.sleep necessary to cool down CPU. System is not fast enough to handle so much locking (kernel switches)
            # time.sleep(0.2)

            try:
                TimedEventHandler().syncBarrier()
            except threading.BrokenBarrierError:
                pass  # will happen at program end

            # a = datetime.datetime.now()
            if not self._wakeUp.wait(self._timeOut):
                print("[Error][CarlaActor::_actorThread] WakeUp Timeout")
                self._isRunning = False
            # TODO possibility unbounded

            self._wakeUp.clear()
            # b = datetime.datetime.now()
            # c = b-a
            # print(c.microseconds)

        print (self._name, "stopped acting")
