#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import datetime
import rospy
import select
import sys
import termios
import threading
import time
import tty

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
        self.__wakeUpOnScenarioEnd = threading.Event()

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
            status = actor.connectToSimulatorAndEvenHandler(self.__simulatorIP, self.__simulatorPort, self.__simulatorTimeout, self.__wakeUpOnScenarioEnd)
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
        isAllRunning = True
        for actor in self.__actors:
            isAllRunning = isAllRunning and actor.getIsRunning()
        
        if isAllRunning:
            startTime = datetime.datetime.now()
            print("# test started at", startTime)
            print("# test running ...")
            self.__wakeUpOnScenarioEnd.clear()
            waitForKeyboardThread = threading.Thread(target=self._wakeUpOnKeyPress)
            waitForKeyboardThread.start()
            self.__wakeUpOnScenarioEnd.wait()
            endTime = datetime.datetime.now()
            print("# test stopped at", endTime, "runtime was", endTime-startTime)
            waitForKeyboardThread.join()
        else:
            print("# test failed starting")

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
    
    def _wakeUpOnKeyPress(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            i = 0
            while True:
                if self.__wakeUpOnScenarioEnd.is_set():
                    break
                
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    c = sys.stdin.read(1)
                    if c == 'q' or c == '\x1b':     # x1b is ESC
                        if not self.__wakeUpOnScenarioEnd.is_set():
                            self.__wakeUpOnScenarioEnd.set()
                        break
                    else:
                        print("# press <q> or <ESC> to stop")
                time.sleep(0.1)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



    def __getConfigDictFromFile(self, fileName):
        print("[WARNING][TestControl::getConfigDictFromFile] Not yet implemented")

        return False
