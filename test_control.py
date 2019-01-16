#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import rospy
import sys
import time

from support.control import InputController
from support.present import ClockHandler, MondeoPlayerAgentHandler
from scenario_parser import OpenScenarioParser
from simulator_control import CarlaSimulatorControl
from timed_event_handler import TimedEventHandler


class TestControl():
    def __init__(self, simulatorType, simulatorIP, simulatorPort, simulatorTimeout, scenarioFileType):
        self.__actors = []
        self.__scenarioParser = None
        self.__simulatorControl = None
        self.__logProcessor = None
        self.__simulatorIP = simulatorIP
        self.__simulatorPort = simulatorPort
        self.__simulatorTimeout = simulatorTimeout

        if scenarioFileType == "OpenScenario":
            self.__scenarioParser = OpenScenarioParser()
        else:
            raise NotImplementedError("Scenarios of Type \"" + scenarioFileType + "\" are not yet supported")

        if simulatorType == "Carla":
            self.__simulatorControl = CarlaSimulatorControl(simulatorIP, simulatorPort, simulatorTimeout)
        else:
            raise NotImplementedError("Simulator of Type \"" + simulatorType + "\" is not yet supported")

        # ROS part, not so fancy yet
        rospy.init_node('control_listener', anonymous=True)
        ClockHandler()
        InputController()
        MondeoPlayerAgentHandler()

    # parses config; returns on error
    def setupTestWithConfig(self, fileName):
        # prepare system
        print("# prepare system")
        TimedEventHandler().clear()

        # parse scenario
        print("# parse scenario")
        if not self.__scenarioParser.parseScenario(fileName):
            return False
        self.__actors = self.__scenarioParser.getActors()
        # self.__scenarioParser.getSceneDescription()
        # self.__scenarioParser.getSimTimeEvents()
        # self.__scenarioParser.getStateEvents()

        # setup carla
        print("# setup carla")
        if not self.__simulatorControl.connect():
            return False

        # setup world
        print("# setup world - skipped")
        # TODO load Scene

        # setup actors
        print("# setup actors")
        isAllActorsConnected = True
        for actor in self.__actors:
            status = actor.connectToSimulatorAndEvenHandler(
                self.__simulatorIP, self.__simulatorPort, self.__simulatorTimeout)
            isAllActorsConnected = isAllActorsConnected and status
        if not isAllActorsConnected:
            return False

        # setup timedEventHandler
        print("# start timedEventHandler - skipped events")

        return True

    def executeTest(self):
        # run timedEventHandler
        print("# run timedEventHandler")

        # run actors
        print("# run actors")
        for actor in self.__actors:
            actor.startActing()

        # run Test - implement logic
        print("# run Test - implement logic!!!")
        input("Press <Enter> to stop simulation")

        # stop actors
        print("# stop actors")
        for actor in self.__actors:
            actor.stopActing()

        # stop timedEventHandler
        print("# stop timedEventHandler - skipped events")
        TimedEventHandler().stop()

        return False

    def cleanupTest(self):
        # cleanup timedEventHandler
        print("# cleanup timedEventHandler - skipped (waiting for sync mode)")

        # cleanup actors
        print("# cleanup actors")
        for actor in self.__actors:
            actor.disconnectFromSimulatorAndEventHandler()

        # cleanup world
        print("# setup world - skipped")
        # TODO default Scene ?

        # cleanup carla
        print("# cleanup carla")
        self.__simulatorControl.disconnect()

        return True

    def __getConfigDictFromFile(self, fileName):
        print("[WARNING][TestControl::getConfigDictFromFile] Not yet implemented")

        return False
