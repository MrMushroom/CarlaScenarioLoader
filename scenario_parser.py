#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import sys
import xmlschema

from pprint import pprint

from support.singleton import Singleton
from support.actor import CarlaActor


class ScenarioParser:
    __metaclass__ = Singleton

    def __init__(self, scenarioFormatFilePath):
        self._scenarioFormatFilePath = scenarioFormatFilePath

        self._scenarioDictionary = {}

        self._actors = []
        self._simTimeEvents = []
        self._stateEvents = []
        self._sceneDescription = None

    def getActors(self):
        return self._actors

    def getSimTimeEvents(self):
        return self._simTimeEvents

    def getStateEvents(self):
        return self._stateEvents

    def getSceneDescription(self):
        return self._sceneDescription

    @abc.abstractmethod
    def parseScenario(self, scenarioDescriptionFilePath):
        pass

    @abc.abstractmethod
    def _processCatalogs(self):
        pass

    @abc.abstractmethod
    def _processActors(self):
        pass

    @abc.abstractmethod
    def _processEntityEvents(self):
        pass

    @abc.abstractmethod
    def _processSimTimeEvents(self):
        pass

    @abc.abstractmethod
    def _processStateEvents(self):
        pass

    @abc.abstractmethod
    def _processSceneDescription(self):
        pass


class OpenScenarioParser(ScenarioParser):
    def __init__(self):
        ScenarioParser.__init__(self, "schema/OpenSCENARIO_v0.9.1.xsd")
        print("[INFO] loaded schema/OpenSCENARIO_v0.9.1.xsd into ScenarioParser")

        self.__oscSchema = None

    def __loadSchema(self):
        try:
            self.__oscSchema = xmlschema.XMLSchema(self._scenarioFormatFilePath)
            return True
        except:
            return False

    def __validateOSC(self, scenarioDescriptionFilePath):
        try:
            return self.__oscSchema.is_valid(scenarioDescriptionFilePath)
        except:
            return False

    def __loadScenarioDictionary(self, scenarioDescriptionFilePath):
        try:
            self._scenarioDictionary = self.__oscSchema.to_dict(scenarioDescriptionFilePath)
            return True
        except:
            return False

    def parseScenario(self, scenarioDescriptionFilePath):
        if not self.__loadSchema():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during XSD loading")
            return False

        if not self.__validateOSC(scenarioDescriptionFilePath):
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during XML validation")
            return False

        if not self.__loadScenarioDictionary(scenarioDescriptionFilePath):
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during dictionary loading")
            return False

        if not self._processCatalogs():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during catalog processing")
            return False

        if not self._processActors():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during actor processing")
            return False

        if not self._processEntityEvents():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during EntityEvent processing")
            return False

        if not self._processSimTimeEvents():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during SimTimeEvent processing")
            return False

        if not self._processStateEvents():
            print("[INFO][ScenarioParser::parseScenario] Unexpected error during StateEvents processing")
            return False

        return True

    def _processCatalogs(self):
        print("[INFO][ScenarioParser::_processCatalogs] Catalogs are not supported yet")
        return True

    def _processActors(self):
        # inform about unsupported parts
        if("Selection" in self._scenarioDictionary["Entities"]):
            print("[INFO][ScenarioParser::_processActors] Selections not supported yet")

        # go parse
        try:
            for entity in self._scenarioDictionary["Entities"]["Object"]:
                if "CatalogReference" in entity:
                    print("[INFO][ScenarioParser::_processActors] CatalogReference not supported yet")
                elif "Vehicle" in entity:
                    actor = CarlaActor(entity["Vehicle"]["@category"], entity["@name"])
                    self._actors.append(actor)
                elif "Pedestrian" in entity:
                    print("[INFO][ScenarioParser::_processActors] Pedestrian not supported yet")
                elif "MiscObject" in entity:
                    print("[INFO][ScenarioParser::_processActors] MiscObject not supported yet")

        except:
            print("[Error][ScenarioParser::_processActors] Unexpected error:", sys.exc_info())
            return False

        return True

    def _processEntityEvents(self):
        raise NotImplementedError("implement processEntityEvents")

    def _processSimTimeEvents(self):
        raise NotImplementedError("implement processSimTimeEvents")

    def _processStateEvents(self):
        raise NotImplementedError("implement processStateEvents")

    def _processSceneDescription(self):
        raise NotImplementedError("implement processSceneDescription")
