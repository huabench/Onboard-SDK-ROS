#! /usr/bin/env python
# coding=utf-8
import rospy
import math

class MeasureData:
    def __init__(self):
        # measure
        self.measure_time_stamp = rospy.Time.now()
        self.measure_center_x =  0.
        self.measure_center_y =  0.
        self.measure_max_length =  0.
        self.measure_gim_pitch =  0.
        self.measure_gim_yaw =  0.
        self.measure_gim_roll =  0.
        self.measure_UAV_x =  0.
        self.measure_UAV_y =  0.
        self.measure_UAV_z =  0.
        self.measure_UAV_vx =  0.
        self.measure_UAV_vy =  0.
        self.measure_laser_dis =  0.

        self._orig_lat = 30.1271400452
        self._orig_lon = 120.082305908
        self._orig_alt = 0.
        self._c_earth = 6378137.0
    
    def update(self, msg):
        self.measure_time_stamp = rospy.Time.now()
        self.measure_center_x = msg.center_x
        self.measure_center_y = msg.center_y
        self.measure_max_length = msg.max_length
        self.measure_gim_pitch = msg.pitch
        self.measure_gim_yaw = msg.yaw
        self.measure_gim_roll = msg.roll
        self.measure_UAV_z = msg.UAV_alt - self._orig_alt
        self.measure_UAV_vx = msg.UAV_vx
        self.measure_UAV_vy = msg.UAV_vy
        self.measure_laser_dis = msg.laser_dis

        d_lat = msg.UAV_lat - self._orig_lat
        d_lon = msg.UAV_lon -self._orig_lon
        self.measure_UAV_x = math.radians(d_lat) * self._c_earth
        self.measure_UAV_y = math.radians(d_lon) * self._c_earth * math.cos(math.radians(msg.UAV_lat))