#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import sys
import xmlschema

from support.singleton import Singleton
from scenario_parser import ScenarioParser
from simulator_control import CarlaSimulatorControl

class TestControl:
  __metaclass__ = Singleton

  def __init__(self, simulatorType):
    if simulatorType == "Carla":
      self.__simulatorControl = CarlaSimulatorControl()
    else:
      self.__simulatorControl = None
      print "[Warning] Simulator \"" + simulatorType + "\" is not yet supported"
    

    self.__actors_ = []
    self.__scenarioParser = None
    self.__timedEventHandler = None
    self.__logProcessor = None

    self.__oscSchemaPath = "schema/OpenSCENARIO_v0.9.1.xsd"
    self.__oscSchema = xmlschema.XMLSchema(self.__oscSchemaPath)

  # parses config; returns on error
  def setupTestWithConfig(self, fileName):
    try:
      if not self.__oscSchema.is_valid(fileName):
        print "[INFO] \"" + fileName + " can not be validated against \"" + self.__oscSchemaPath + "\""
        return False
    except:
      print "[INFO][TestControl::setupTestWithConfig] Unexpected error during schema validation"
      return False

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