#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc
import sys
import xmlschema

from pprint import pprint

from support.actor import CarlaActor
from support.util import Action, Pose


class ScenarioParser():
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
        # parse Entities
        # inform about unsupported parts
        if("Selection" in self._scenarioDictionary["Entities"]):
            print("[INFO][ScenarioParser::_processActors] Selections not supported yet")

        # go parse Entities
        try:
            for entity in self._scenarioDictionary["Entities"]["Object"]:
                # choice CatalogReference, Vehicle, Pedestrian, MiscObject
                if "CatalogReference" in entity:
                    print("[WARNING][ScenarioParser::_processActors] CatalogReference not parsed. Guessing Vehicle:", entity["@name"])
                    self._actors.append(CarlaActor("Vehicle", entity["@name"]))
                elif "Vehicle" in entity:
                    self._actors.append(CarlaActor("Vehicle", entity["@name"]))
                elif "Pedestrian" in entity:
                    print("[INFO][ScenarioParser::_processActors] Pedestrian not supported yet")
                elif "MiscObject" in entity:
                    print("[INFO][ScenarioParser::_processActors] MiscObject not supported yet")

                # Controller
                if "Controller" in entity:
                    print("[INFO][ScenarioParser::_processActors] Controller not supported yet")

        except:
            print("[Error][ScenarioParser::_processActors] Unexpected error:", sys.exc_info())
            return False

        # parse Storyboard-Init
        if "Global" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            print("[INFO][ScenarioParser::_processActors] Init:Action:Global not supported yet")
        if "UserDefined" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            print("[INFO][ScenarioParser::_processActors] Init:Action:UserDefined not supported yet")
        if "Private" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            for action in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]["Private"]:
                for actor in self._actors:
                    if actor.getName() == action["@object"]:
                        # parse the action and set speed and pose for actor
                        speed, pose = self._parseSpeedAndPoseFromAction(action["Action"])
                        actor.setInit(speed, pose)
        return True

    def _processEntityEvents(self):
        try:
            if(len(self._scenarioDictionary["Storyboard"]["Story"]) != 1):
                print("[INFO][ScenarioParser::_processActors] Use Exactly one Story")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"]) != 1):
                print("[INFO][ScenarioParser::_processActors] Use Exactly one Act")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"]) != 1):
                print("[INFO][ScenarioParser::_processActors] Use Exactly one ConditionGroup")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"]) != 1):
                print("[INFO][ScenarioParser::_processActors] Use Exactly one Condition")
                return False
            if(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"][0]["ByValue"]["SimulationTime"]["@rule"] != "equal_to" or
                    self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"][0]["ByValue"]["SimulationTime"]["@value"] != 0.0):
                print("[INFO][ScenarioParser::_processActors] unsupported starting condition")
                return False

            # TODO parse actor behavior here
            print("# TODO parse actor behavior here")
            return True
        except:
            print("[Error][ScenarioParser::_processEntityEvents] Unexpected error:", sys.exc_info())
            return False

        return True

    def _processSimTimeEvents(self):
        # TODO process SimTimeEvents here
        print("# TODO process SimTimeEvents here")
        return True

    def _processStateEvents(self):
        # TODO process StateEvents here
        print("# TODO process StateEvents here")
        return True

    def _processSceneDescription(self):
        # TODO process SceneDescription here
        print("# TODO process SceneDescription here")
        return True

    def _parseSpeedAndPoseFromAction(self, action):
        try:
            speed = action[0]["Longitudinal"]["Speed"]["Target"]["Absolute"]["@value"]
            pose = Pose(action[1]["Position"]["World"]["@x"],
                        action[1]["Position"]["World"]["@y"],
                        action[1]["Position"]["World"]["@z"],
                        action[1]["Position"]["World"]["@r"],
                        action[1]["Position"]["World"]["@p"],
                        action[1]["Position"]["World"]["@h"])

            return speed, pose
        except:
            print("[Error][ScenarioParser::_parseSpeedAndPoseFromAction] Unexpected error:", sys.exc_info())
            return 0.0, None
