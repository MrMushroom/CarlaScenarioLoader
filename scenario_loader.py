#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import inspect
import os
import sys

from test_control import TestControl

def main():

  if len(sys.argv) == 2:
    if sys.argv[1] == "help":
      print inspect.getfile(inspect.currentframe()) + " help - get this message"
      print inspect.getfile(inspect.currentframe()) + " <Carla> <OpenScenario-file> - for standard execution"
      exit()
    else:
      print "[Error] Wrong command line parameters"
      exit()
  elif len(sys.argv) == 3:
    if os.path.isfile(sys.argv[2]):
      # create TestControl class
      testControl = TestControl()
      # try to load Config
      if not testControl.setupTestWithConfig(sys.argv[1], sys.argv[2]):
        print "[Error] TestControl-Setup failed"
        exit()
      testControl.startSimulation()
      # Simulation is built up
      # Simulation is executed
      testControl.stopSimulation()
      # Simulation environment is cleaned
      exit()

    else:
      print "[Error] second parameter has to be a valid filepath"
      exit()
  else:
    print "[Error] try \"" + inspect.getfile(inspect.currentframe()) + " help\""

if __name__ == '__main__':

    main()
