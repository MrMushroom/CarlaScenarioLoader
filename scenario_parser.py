#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

'''
Main TODO-list
- actor parsing is to much integrated into Carla. uses CarlaActor
- consider moving EntityEvent handling into a separate event handler (startCondition handling difficulties when using multiple triggering entities)
'''

import abc
import sys
import xmlschema

from pprint import pprint

from support.actor import CarlaActor
from support.events import EntityEvent, StartCondition
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
            self.__oscSchema = xmlschema.XMLSchema(
                self._scenarioFormatFilePath)
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
            self._scenarioDictionary = self.__oscSchema.to_dict(
                scenarioDescriptionFilePath)
            return True
        except:
            return False

    def parseScenario(self, scenarioDescriptionFilePath):
        if not self.__loadSchema():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during XSD loading")
            return False

        if not self.__validateOSC(scenarioDescriptionFilePath):
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during XML validation")
            return False

        if not self.__loadScenarioDictionary(scenarioDescriptionFilePath):
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during dictionary loading")
            return False

        if not self._processCatalogs():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during catalog processing")
            return False

        if not self._processActors():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during actor processing")
            return False

        if not self._processEntityEvents():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during EntityEvent processing")
            return False

        if not self._processSimTimeEvents():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during SimTimeEvent processing")
            return False

        if not self._processStateEvents():
            print(
                "[INFO][ScenarioParser::parseScenario] Unexpected error during StateEvents processing")
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
                    print(
                        "[WARNING][ScenarioParser::_processActors] CatalogReference not parsed. Guessing Vehicle:", entity["@name"])
                    self._actors.append(CarlaActor("Vehicle", entity["@name"]))
                elif "Vehicle" in entity:
                    self._actors.append(CarlaActor("Vehicle", entity["@name"]))
                elif "Pedestrian" in entity:
                    print(
                        "[INFO][ScenarioParser::_processActors] Pedestrian not supported yet")
                elif "MiscObject" in entity:
                    print(
                        "[INFO][ScenarioParser::_processActors] MiscObject not supported yet")

                # Controller
                if "Controller" in entity:
                    print(
                        "[INFO][ScenarioParser::_processActors] Controller not supported yet")

        except:
            print(
                "[Error][ScenarioParser::_processActors] Unexpected error:", sys.exc_info())
            return False

        # parse Storyboard-Init
        if "Global" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            print(
                "[INFO][ScenarioParser::_processActors] Init:Action:Global not supported yet")
        if "UserDefined" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            print(
                "[INFO][ScenarioParser::_processActors] Init:Action:UserDefined not supported yet")
        if "Private" in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]:
            for action in self._scenarioDictionary["Storyboard"]["Init"]["Actions"]["Private"]:
                for actor in self._actors:
                    if actor.getName() == action["@object"]:
                        # parse the action and set speed and pose for actor
                        speed, pose = self._parseSpeedAndPoseFromInitAction(
                            action["Action"])
                        actor.setInit(speed, pose)
        return True

    def _processEntityEvents(self):
        try:
            if(len(self._scenarioDictionary["Storyboard"]["Story"]) != 1):
                print(
                    "[INFO][ScenarioParser::_processEntityEvents] Use Exactly one Story")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"]) != 1):
                print(
                    "[INFO][ScenarioParser::_processEntityEvents] Use Exactly one Act")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"]) != 1):
                print(
                    "[INFO][ScenarioParser::_processEntityEvents] Use Exactly one ConditionGroup")
                return False
            if(len(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"]) != 1):
                print(
                    "[INFO][ScenarioParser::_processEntityEvents] Use Exactly one Condition")
                return False
            if(self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"][0]["ByValue"]["SimulationTime"]["@rule"] != "equal_to" or
                    self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Conditions"]["Start"]["ConditionGroup"][0]["Condition"][0]["ByValue"]["SimulationTime"]["@value"] != 0.0):
                print(
                    "[INFO][ScenarioParser::_processEntityEvents] unsupported starting condition")
                return False

            # TODO INFO: starting condition = simulation time equals 0. So no TimedEventHandler necessary for now

            for sequence in self._scenarioDictionary["Storyboard"]["Story"][0]["Act"][0]["Sequence"]:
                # TODO sequenceName should be unique
                sequenceName = sequence["@name"]
                if(sequence["@numberOfExecutions"] != 1):
                    print("[INFO][ScenarioParser::_processEntityEvents]",
                          sequenceName, "Exactly one execution per sequence supported")
                    return False
                if(len(sequence["Actors"]["Entity"]) != 1):
                    print("[INFO][ScenarioParser::_processEntityEvents]",
                          sequenceName, "Exactly one Actor per Sequence supported")
                    return False
                if(len(sequence["Maneuver"]) != 1):
                    print("[INFO][ScenarioParser::_processEntityEvents]",
                          sequenceName, "Exactly one Maneuver per Sequence supported")
                    return False

                for event in sequence["Maneuver"][0]["Event"]:
                    # eventName = event["@name"]
                    parsedAction = None
                    for action in event["Action"]:
                        # actionName = action["@name"]
                        parsedAction = self._processAction(action)
                        if parsedAction == None:
                            print(
                                "[Error][ScenarioParser::_processEntityEvents] failed _processAction")
                            return False

                    parsedStartCondition = StartCondition()
                    if self._processStartCondition(event, parsedStartCondition) == False:
                        print(
                            "[Error][ScenarioParser::_processEntityEvents] failed _processStartCondition")
                        return False

                    actors = [actor
                              for actor in self._actors
                              if actor.getName() == sequence["Actors"]["Entity"][0]["@name"]]
                    entityEvent = EntityEvent(
                        parsedAction, actors, parsedStartCondition)

                    for actor in actors:
                        actor.addEntityEvent(entityEvent)

            return True
        except:
            print(
                "[Error][ScenarioParser::_processEntityEvents] Unexpected error:", sys.exc_info())
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

    def _processAction(self, action):
        parsedAction = Action()

        try:
            if("Private" in action):
                if("Longitudinal" in action["Private"]):
                    parsedAction.longitudinal_speed = action["Private"][
                        "Longitudinal"]["Speed"]["Target"]["Absolute"]["@value"]
                    parsedAction.longitudinal_dynamics_shape = action[
                        "Private"]["Longitudinal"]["Speed"]["Dynamics"]["@shape"]
                    parsedAction.longitudinal_dynamics_rate = action[
                        "Private"]["Longitudinal"]["Speed"]["Dynamics"]["@rate"]
                else:
                    print(
                        "[Error][ScenarioParser::_parseAction] Unsupported Private action:", action["Private"].keys())
                    return None
            else:
                print(
                    "[Error][ScenarioParser::_parseAction] Only Private actions supported for now")
                return None

        except:
            print(
                "[Error][ScenarioParser::_parseAction] Unexpected error:", sys.exc_info())
            return None

        return parsedAction

    def _processStartCondition(self, event, startCondition):
        if(len(event["StartConditions"]["ConditionGroup"]) != 1):
            print(
                "[INFO][ScenarioParser::_processStartCondition] Use Exactly one ConditionGroup")
            return False
        if(len(event["StartConditions"]["ConditionGroup"][0]["Condition"]) != 1):
            print(
                "[INFO][ScenarioParser::_processStartCondition] Use Exactly one Condition")
            return False

        startCondition.priority = event["@priority"]
        startCondition.delay = event["StartConditions"]["ConditionGroup"][0]["Condition"][0]["@delay"]
        startCondition.edge = event["StartConditions"]["ConditionGroup"][0]["Condition"][0]["@edge"]

        condition = event["StartConditions"]["ConditionGroup"][0]["Condition"][0]
        if "ByEntity" in condition:
            if(len(condition["ByEntity"]["TriggeringEntities"]["Entity"]) != 1):
                print(
                    "[INFO][ScenarioParser::_processStartCondition] Use Exactly one triggering entity")
                return False
            # ignore condition["ByEntity"]["TriggeringEntities"]["@rule"]
            startCondition.triggeringEntity = condition["ByEntity"]["TriggeringEntities"]["Entity"][0]["@name"]

            if "ReachPosition" in condition["ByEntity"]["EntityCondition"]:
                startCondition.pose_tolerance = condition["ByEntity"][
                    "EntityCondition"]["ReachPosition"]["@tolerance"]
                startCondition.pose = Pose(condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@x"],
                                           condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@y"],
                                           condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@z"],
                                           condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@r"],
                                           condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@p"],
                                           condition["ByEntity"]["EntityCondition"]["ReachPosition"]["Position"]["World"]["@h"])
            else:
                print("[INFO][ScenarioParser::_processStartCondition] Unsupported EntityCondition:",
                      condition["ByEntity"]["EntityCondition"].keys())
                return False
        else:
            print(
                "[Error][ScenarioParser::_processStartCondition] Unsupported Condition:", condition.keys())
            return False

        return True

    def _parseSpeedAndPoseFromInitAction(self, action):
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
            print(
                "[Error][ScenarioParser::_parseSpeedAndPoseFromInitAction] Unexpected error:", sys.exc_info())
            return 0.0, None
