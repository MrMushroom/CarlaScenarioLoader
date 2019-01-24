#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import inspect
import os
import prctl
import sys

from test_control import TestControl

CMDLINEPARAM_PROGNAME = 0
CMDLINEPARAM_PROGMODE = 1
CMDLINEPARAM_SIMTYPE = 1
CMDLINEPARAM_SIMIP = 2
CMDLINEPARAM_SIMPORT = 3
CMDLINEPARAM_SIMTIMEOUT = 4
CMDLINEPARAM_SCENARIOFORMAT = 5
CMDLINEPARAM_SCENARIOFILES_START = 6

def main():

    if len(sys.argv) == 2:
        if sys.argv[CMDLINEPARAM_PROGMODE] == "help":
            print(inspect.getfile(inspect.currentframe()) + " help - get this message")
            print(inspect.getfile(inspect.currentframe()) +
                  " <Carla> <OpenScenario> <OpenScenario-file> - for standard execution")
            exit()
        else:
            print("[Error] Wrong command line parameters, try: \"" +
                  inspect.getfile(inspect.currentframe()) + " help\"")
            exit()
    # TODO implement multi scenario
    elif len(sys.argv) > CMDLINEPARAM_SCENARIOFILES_START:
        # load params
        simType = sys.argv[CMDLINEPARAM_SIMTYPE]
        simIP = sys.argv[CMDLINEPARAM_SIMIP]
        simPort = int(sys.argv[CMDLINEPARAM_SIMPORT])
        simTimeout = float(sys.argv[CMDLINEPARAM_SIMTIMEOUT])
        scenarioFormat = sys.argv[CMDLINEPARAM_SCENARIOFORMAT]
        scenarioFiles = getFileNames()

        if len(scenarioFiles) == 0:
            print("[Error] parameter", CMDLINEPARAM_SCENARIOFILES_START, "-", len(sys.argv), "have to contain valid paths to files")
            exit()

        for scenarioFile in scenarioFiles:
            print("## Loading Scenario:", scenarioFile)

            # create TestControl class
            testControl = TestControl(simType, simIP, simPort, simTimeout, scenarioFormat)

            # try loading scenario-config
            if not testControl.setupTestWithConfig(scenarioFile):
                print("[Error] TestControl-Setup failed for", scenarioFile)
                # TODO save log?
                continue

            testControl.executeTest()
            # Simulation is built up
            # Simulation is executed
            testControl.cleanupTest()

            if testControl.isSkipCurrentTest:
                print("## Scenario:", scenarioFile, "skipped.")
                continue
            elif testControl.isAbortAllFurtherTests:
                print("## Scenario:", scenarioFile, "and all further scenarios aborted")
                break

            print("##Finished Scenario:", scenarioFile)

        print(" --- The End ---")
    
    else:
        print("[Error] try \"" + inspect.getfile(inspect.currentframe()) + " help\"")

def getFileNames():
    scenarioFiles = []
    
    for i in range(CMDLINEPARAM_SCENARIOFILES_START, len(sys.argv)):
        if os.path.isfile(sys.argv[i]):
            scenarioFiles.append(sys.argv[i])
        elif os.path.isdir(sys.argv[i]):
            for root, subdirs, files in os.walk(sys.argv[i]):
                for filename in files:
                    file_path = os.path.join(root, filename)
                    scenarioFiles.append(os.path.join(root, filename))
        else:
            print("[Info][main::getFileNames] Hmm wierd ... what did we find?")

    return scenarioFiles

if __name__ == '__main__':
    prctl.set_name("init")
    main()
