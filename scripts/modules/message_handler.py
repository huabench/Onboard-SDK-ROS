#! /usr/bin/env python
# coding=utf-8

import rospy
from modules import message
from modules.common_type import GroundCMD

class MessageHandler:
    def __init__(self):
        self.__subsriber_init()

    def __subsriber_init(self):
        self.__est_tar_sub = message.TargetSubscriber('/iusl/estimate_tar_state')
        self.__g_tar_sub = message.TargetSubscriber('/iusl_ros/ground_tar_state')
        self.__g_cmd_sub = message.UInt8Subscriber('/iusl_ros/ground_mission_cmd')
        self.__det_res_sub = message.DetectionResultSubscriber('/iusl/DetectionResult')
        self.__flight_status_sub = message.UInt8Subscriber('/dji_osdk_ros/flight_status')
        self.__rtk_pos_sub = message.RTKPositionSubscriber('/dji_osdk_ros/rtk_position')
        self.__rtk_vel_sub = message.VectorSubscriber('/dji_osdk_ros/rtk_velocity')
        self.__rtk_yaw_sub = message.Int16Subscriber('/dji_osdk_ros/rtk_yaw')    
        self.__gim_angle_sub = message.VectorSubscriber('/dji_osdk_ros/gimbal_angle')
       
        self.__uav_ctrl_cmd_pub = message.UAVCtlCmdPublisher()

    def service_setup(self, name, type, data):
        rospy.wait_for_service(name)
        srv_h = rospy.ServiceProxy(name, type)
        res = srv_h(data)

    def get_ground_cmd(self):
        cmd = self.__g_cmd_sub.num
        self.__g_cmd_sub.num = 0 # make it none when get it
        return cmd

    def get_flight_status(self):
        return self.__flight_status_sub.num

    def publish_ctrl_cmd(self, data):
        self.__uav_ctrl_cmd_pub.publish(data)


