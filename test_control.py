#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import sys

from support.singleton import Singleton
from scenario_parser import OpenScenarioParser
from simulator_control import CarlaSimulatorControl


class TestControl:
    __metaclass__ = Singleton

    def __init__(self, simulatorType, simulatorIP, scenarioFileType):
        self.__actors = []
        self.__scenarioParser = None
        self.__simulatorControl = None
        self.__timedEventHandler = None
        self.__logProcessor = None
        self.__simulatorIP = simulatorIP
        self.__simulatorPort = 2000
        self.__simulatorTimeout = 2.0

        if scenarioFileType == "OpenScenario":
            self.__scenarioParser = OpenScenarioParser()
        else:
            raise NotImplementedError("Scenarios of Type \"" + scenarioFileType + "\" are not yet supported")

        if simulatorType == "Carla":
            self.__simulatorControl = CarlaSimulatorControl(simulatorIP)
        else:
            raise NotImplementedError("Simulator of Type \"" + simulatorType + "\" is not yet supported")

    # parses config; returns on error
    def setupTestWithConfig(self, fileName):
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
        self.__simulatorControl.connect()

        # setup world
        print("# setup world - skipped")
        # TODO load Scene

        # setup actors
        print("# setup actors")
        for actor in self.__actors:
            actor.connectToSimulatorAndEvenHandler(
                self.__simulatorIP, self.__simulatorPort, self.__simulatorTimeout, self.__timedEventHandler)

        # setup timedEventHandler
        print("# setup timedEventHandler - skipped (waiting for sync mode)")

        return True

    def executeTest(self):
        print("[WARNING][TestControl::startSimulation] Not yet implemented")

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
