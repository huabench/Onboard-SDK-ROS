#! /usr/bin/env python
# coding=utf-8
from enum import Enum, unique # Python 2.7 users need to have 'enum34' installed


class VehicleStates(Enum):
    ONGROUND = 0
    FLYING = 1
    ASCENDING = 7
    SEARCHING = 2
    AIMMING = 3
    CONTROLLING = 4
    REJECTED = 5
    RETURNING = 6


print VehicleStates.AIMMING.value == 3
