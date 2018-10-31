#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from support.singleton import Singleton
from scenario_parser import ScenarioParser
from simulator_control import CarlaSimulatorControl

class TestControl:
  __metaclass__ = Singleton

  def __init__(self):
    self.__actors_ = []
    self.__scenarioParser = None
    self.__simulatorControl = None
    self.__timedEventHandler = None
    self.__logProcessor = None

  def setupTestWithConfig(self, simulatorType, fileName):
    # set simulator control; return on error
    if simulatorType == "Carla":
      self.__simulatorControl = CarlaSimulatorControl
    else:
      print "[Warning] Simulator \"" + simulatorType + "\" is not yet supported"
      return False

    # parse config; return on error
    print "[Warning] Scenario parsing not yet implemented"

    
    return False

  def startSimulation(self):
    print "[WARNING][startSimulation] Not yet implemented"
    
    return False

  def stopSimulation(self):
    print "[WARNING][stopSimulation] Not yet implemented"
    
    return False

  def __getConfigDictFromFile(self, fileName):
    print "[WARNING][getConfigDictFromFile] Not yet implemented"
    
    return False