#! /usr/bin/env python
# coding=utf-8

from enum import Enum, unique # Python 2.7 users need to have 'enum34' installed

@unique
class VehicleStates(Enum):
    ONGROUND = 0
    FLYING = 1
    ASCENDING = 7
    SEARCHING = 2
    AIMMING = 3
    CONTROLLING = 4
    REJECTED = 5
    RETURNING = 6


@unique
# 1 takeoff, 2 search, 3 aim, 4 control 5 reject 6 return
class GroundCMD(Enum):
    NONE = 0
    TAKEOFF = 1
    SEARCH = 2
    AIM = 3
    CONTROL = 4
    REJECT = 5
    RETURN  =6

@unique
class FlightState(Enum):
    GROUND = 0
    MID = 1
    AIR = 2

@unique
class CtlCMD(Enum):
    NONE = 0
    TAKEOFF = 1
    SEARCH = 2
    AIM = 3
    CONTROL = 4
    RETURN = 6
