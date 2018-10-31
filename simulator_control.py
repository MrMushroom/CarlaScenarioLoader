#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from support.singleton import Singleton

class SimulatorControl:
  __metaclass__ = Singleton


class CarlaSimulatorControl(SimulatorControl):
  pass
