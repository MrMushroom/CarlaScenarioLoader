#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import sys
import time

from collections import deque

from .util import Action, Pose, TimeStamp


def constantStraightAhead(pose, timestamp, speed):
    queue = deque()
    if speed >= 0:
        s = 0.1  # 100mm
        v = speed * 1000.0 / 3600
        dt = s / v

        yaw = pose.getOrientation()[2]
        pitch = pose.getOrientation()[1]
        dx = s*math.cos(yaw)*math.cos(pitch)
        dy = s*math.sin(yaw)*math.cos(pitch)
        dz = s*math.sin(pitch)

        sec, usec = timestamp.getInt()

        # reference if next point is too far away
        queue.append(Action(pose, timestamp))
        # pointCount < 100 (99) guarantees 0.5m z difference for streets with 11.5% elevation, as long as the car is parallel to the street
        pointCount = 1
        while(pointCount < 100):
            newPose = Pose(x=pose.getPosition()[0] + dx*pointCount,
                           y=pose.getPosition()[1] + dy*pointCount,
                           z=pose.getPosition()[2] + dz*pointCount,
                           roll=pose.getOrientation()[0],
                           pitch=pose.getOrientation()[1],
                           yaw=pose.getOrientation()[2])
            newTimestamp = TimeStamp(sec, usec)
            newTimestamp.addFloat(dt*pointCount)

            queue.append(Action(newPose, newTimestamp))
            pointCount += 1
    else:
        pass

    return queue


def interpolateActions(a1, a2, timestamp):
    dt = a2.timestamp.getFloat()-a1.timestamp.getFloat()
    dx = a2.pose.getPosition()[0] - a1.pose.getPosition()[0]
    dy = a2.pose.getPosition()[1] - a1.pose.getPosition()[1]
    dz = a2.pose.getPosition()[2] - a1.pose.getPosition()[2]
    droll = a2.pose.getOrientation()[0] - a1.pose.getOrientation()[0]
    dpitch = a2.pose.getOrientation()[1] - a1.pose.getOrientation()[1]
    dyaw = a2.pose.getOrientation()[2] - a1.pose.getOrientation()[2]

    idt = timestamp.getFloat() - a1.timestamp.getFloat()
    idx = dx / dt * idt
    idy = dy / dt * idt
    idz = dz / dt * idt
    idroll = droll / dt * idt
    idpitch = dpitch / dt * idt
    idyaw = dyaw / dt * idt

    iPose = Pose(   x=a1.pose.getPosition()[0] + idx,
                    y=a1.pose.getPosition()[1] + idy,
                    z=a1.pose.getPosition()[2] + idz,
                    roll=a1.pose.getOrientation()[0] + idroll,
                    pitch=a1.pose.getOrientation()[1] + idpitch,
                    yaw=a1.pose.getOrientation()[2] + idyaw)

    return iPose
