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

    def __init__(self, simulatorType, scenarioFileType):
        self.__actors_ = []
        self.__scenarioParser = None
        self.__simulatorControl = None
        self.__timedEventHandler = None
        self.__logProcessor = None

        if scenarioFileType == "OpenScenario":
            self.__scenarioParser = OpenScenarioParser()
        else:
            raise NotImplementedError("Scenarios of Type \"" + scenarioFileType + "\" are not yet supported")

        if simulatorType == "Carla":
            self.__simulatorControl = CarlaSimulatorControl()
        else:
            raise NotImplementedError("Simulator of Type \"" + simulatorType + "\" is not yet supported")

    # parses config; returns on error
    def setupTestWithConfig(self, fileName):
        if not self.__scenarioParser.parseScenarioXML(fileName):
            return False

        raise NotImplementedError("Scenario parsing not yet full yimplemented")
        return False

    def startSimulation(self):
        print "[WARNING][TestControl::startSimulation] Not yet implemented"

        return False

    def stopSimulation(self):
        print "[WARNING][TestControl::stopSimulation] Not yet implemented"

        return False

    def __getConfigDictFromFile(self, fileName):
        print "[WARNING][TestControl::getConfigDictFromFile] Not yet implemented"

        return False
