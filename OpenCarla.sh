#!/bin/bash

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#Add UE4_ROOT=~/UnrealEngine_4.19 to your ~/.bashrc for manual execution
cd ~/carla
UE4_ROOT=~/UnrealEngine_4.21 make launch-only
