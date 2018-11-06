#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import abc

class IObserver():
  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def update(self, event):
    pass
