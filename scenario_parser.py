#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import xmlschema

from support.singleton import Singleton

class ScenarioParser:
  __metaclass__ = Singleton

  def __init__(self, scenarioFormatFilePath):
    self.__scenarioFormatFilePath = scenarioFormatFilePath

    self.__scenarioSchema = None
    self.__scenarioDescriptionFilePath = ""

    self.__actors = []
    self.__simTimeEvents = []
    self.__stateEvents = []

  def parseScenarioXML(self, scenarioDescriptionFilePath):
    try:
      self.__scenarioSchema = xmlschema.XMLSchema(self.__scenarioFormatFilePath)
    except:
      print "[INFO][ScenarioParser::parseScenario] Unexpected error during XSD loading"
      return False

    try:
      if not self.__scenarioSchema.is_valid(scenarioDescriptionFilePath):
        print "[INFO][ScenarioParser::parseScenario] \"" + scenarioDescriptionFilePath + " can not be validated against \"" + self.__scenarioFormatFilePath + "\""
        return False
    except:
      print "[INFO][ScenarioParser::parseScenario] Unexpected error during XML validation"
      return False

    return self.processActors() and self.processEntityEvents() and self.processSimTimeEvents() and self.processStateEvents()

  def getActors(self):
    return self.__actors

  def getSimTimeEvents(self):
    return self.__simTimeEvents

  def getStateEvents(self):
    return self.__stateEvents

  def processActors(self):
    raise NotImplementedError("implement processActors")

  def processEntityEvents(self):
    raise NotImplementedError("implement processEntityEvents")

  def processSimTimeEvents(self):
    raise NotImplementedError("implement processSimTimeEvents")

  def processStateEvents(self):
    raise NotImplementedError("implement processStateEvents")
  

class OpenScenarioParser(ScenarioParser):
  def __init__(self):
    ScenarioParser.__init__(self, "schema/OpenSCENARIO_v0.9.1.xsd")
    print "[INFO] loaded schema/OpenSCENARIO_v0.9.1.xsd into ScenarioParser"