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

tolerance = 0.1


def constantStraightAhead(pose, timestamp, speed):
    queue = deque()
    if speed > 0:
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


def trajectory(vertices, domain, pose, timestamp, speed):
    queue = deque()
    if domain == "distance":
        if speed >= 0:
            s = 0.1  # 100mm
            v = speed * 1000.0 / 3600
            dt = s / v

            last_pose = pose
            last_timestamp = timestamp
            distance = 0.0
            for vertex in vertices:
                if (((vertex.reference - tolerance) < distance) and (distance < (vertex.reference + tolerance))):
                    if vertex.shape == vertex.shapeTags["Clothoid"]:
                        print("[Warning] no check for vertex.relativeObject=", vertex.relativeObject, "Estimate self")
                        print("[Warning] ignoring vertex.pose=", vertex.pose, "Estimate all 0.0")
                        print("[Warning] ignoring vertex.positioning=", vertex.positioning, "Estimate relative")
                        print("[Warning] ignoring vertex.orientation=", vertex.orientation, " for Clothoid")
                        print("[Warning] ignoring vertex.clothoid_curvatureDot=", vertex.clothoid_curvatureDot, "Estimate 0.0")

                        queue += calculatePosesForArc(last_pose, s, last_timestamp, dt, vertex.clothoid_curvature, vertex.clothoid_length)
                        distance = vertex.clothoid_length

                    elif vertex.shape == vertex.shapeTags["Polyline"]:
                        print("[Warning] no check for vertex.relativeObject=", vertex.relativeObject, "Estimate self")

                        if(vertex.positioning == vertex.positioningTags["absolute"]):
                            x = vertex.pose.getPosition()[0]
                            y = vertex.pose.getPosition()[1]
                            z = vertex.pose.getPosition()[2]
                        elif(vertex.positioning == vertex.positioningTags["relative"]):
                            x = pose.getPosition()[0] + vertex.pose.getPosition()[0]
                            y = pose.getPosition()[1] + vertex.pose.getPosition()[1]
                            z = pose.getPosition()[2] + vertex.pose.getPosition()[2]
                        else:
                            print("[Warning] unknown value for vertex.positioning=", vertex.positioning, "-> will crash")

                        if(vertex.orientation == vertex.positioningTags["absolute"]):
                            r = vertex.pose.getOrientation()[0]
                            p = vertex.pose.getOrientation()[1]
                            y = vertex.pose.getOrientation()[2]
                        elif(vertex.orientation == vertex.positioningTags["relative"]):
                            r = pose.getOrientation()[0] + vertex.pose.getOrientation()[0]
                            p = pose.getOrientation()[1] + vertex.pose.getOrientation()[1]
                            y = pose.getOrientation()[2] + vertex.pose.getOrientation()[2]
                        else:
                            print("[Warning] unknown value for vertex.orientation=", vertex.orientation, "-> will crash")

                        newPose = Pose(x=x, y=y, z=z, roll=r, pitch=p, yaw=y)
                        sec, usec = timestamp.getInt()
                        dt = vertex.reference / v
                        newTimestamp = TimeStamp(sec, usec)
                        newTimestamp.addFloat(dt)

                        queue.append(Action(newPose, newTimestamp))

                    else:
                        print("[Error][maneuvers::trajectory]: unknown vertex.shape =", vertex.shape)
                else:
                    print("[Error][maneuvers::trajectory]: distance != reference missmatch", distance, vertex.reference)

        else:
            print("[Error][maneuvers::trajectory]: no trajacetory in standstill")
    else:
        print("[Error][maneuvers::trajectory]: unknown domain", domain)

    return queue


def calculatePosesForArc(pose, stepsize, timestamp, dt, curvature, length):
    queue = deque()
    radius = abs(1.0/curvature)

    if (curvature <= 0):
        startAngle = pose.getOrientation()[2] + math.pi/2.0
    else:
        startAngle = pose.getOrientation()[2] - math.pi/2.0

    circleX = pose.getPosition()[0] + math.cos(startAngle-math.pi) * radius
    circleY = pose.getPosition()[1] + math.sin(startAngle-math.pi) * radius
    hStart = pose.getOrientation()[2]
    x = pose.getPosition()[0]
    y = pose.getPosition()[1]
    h = pose.getOrientation()[2]

    step = 0
    sec, usec = timestamp.getInt()

    while((step*stepsize) < length):
        x_old = x
        y_old = y

        angle = startAngle + (step*stepsize) * curvature
        x = circleX + math.cos(angle)*radius
        y = circleY + math.sin(angle)*radius
        h = math.atan2(abs(y-y_old), abs(x-x_old))
        h = hStart + (step*stepsize) * curvature

        newPose = Pose(x=x,
                       y=y,
                       z=pose.getPosition()[2],
                       roll=pose.getOrientation()[0],
                       pitch=pose.getOrientation()[1],
                       yaw=h)
        newTimestamp = TimeStamp(sec, usec)
        newTimestamp.addFloat(dt*step)
        queue.append(Action(newPose, newTimestamp))

        step = step + 1
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

    iPose = Pose(x=a1.pose.getPosition()[0] + idx,
                 y=a1.pose.getPosition()[1] + idy,
                 z=a1.pose.getPosition()[2] + idz,
                 roll=a1.pose.getOrientation()[0] + idroll,
                 pitch=a1.pose.getOrientation()[1] + idpitch,
                 yaw=a1.pose.getOrientation()[2] + idyaw)

    return iPose
