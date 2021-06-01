#! /usr/bin/env python
# coding=utf-8

from dji_osdk_ros.msg import iuslUAVCtrlCmd    # my control cmd
from modules.common_type import FlightState, VehicleStates, CtlCMD
from modules.message_handler import MessageHandler
import math
import time

class Model:
    def __init__(self):
        self.__ctl_cmd = iuslUAVCtrlCmd()
        self.__mh = MessageHandler()
        self.__setup()
        self.count = 0

    def __setup(self):
        pass
        #self.__mh.service_setup('set_go_home_altitude', SetGoHomeAltitude, 38.)
        #self.__mh.service_setup('obtain_release_control_authority', ObtainControlAuthority, True)

    def __get_flight_state(self):
        self.count += 1
        if self.count > 12:
            return 0
        elif self.count > 10:
            return 1
        elif self.count > 3:
            return 2
        return self.__mh.get_flight_status()

    # callback functions

    def send_ctl_cmd(self):
        #self.__mh.publish_ctrl_cmd(self.__ctl_cmd)
        print self.state, 'send ctl data', CtlCMD(self.__ctl_cmd.task)
        pass

    # conditions
    def is_ascending(self):
        fs = FlightState(self.__get_flight_state())
        # TODO: check if FlightState.GROUND
        #print('check', fs)
        return fs != FlightState.AIR

    def is_descending(self):
        fs = FlightState(self.__get_flight_state())
        # TODO: check if FlightState.GROUND
        #print("check desc")
        return fs != FlightState.GROUND

    # prepare and on_enter functions
    def on_enter_ONGROUND(self):
        print "back to ground ============="

    def prepare_take_off(self):
        print "send take off cmd"
        self.__ctl_cmd.task = CtlCMD.TAKEOFF.value

    def on_enter_ASCENDING(self):
        print "enter ascending"
        self.__ctl_cmd.task = CtlCMD.NONE.value
        time.sleep(0.2)
        self.ascending() # go to ASCENDING state

    def prepare_search(self):
        # TODO: fix member 
        self.__ctl_cmd.task = CtlCMD.SEARCH.value
        '''
        self.__ctl_cmd.mode = 2
        self.__ctl_cmd.z = - ground_tar_z + 3 + my_UAV_home_z

        k = 0.2
        max_speed = 2
        dx = ground_tar_x - my_UAV_x
        dy = ground_tar_y - my_UAV_y
        dis2 = dx**2 + dy**2 + 0.01
        cmd_vel_x = ground_tar_vx + k * (dis2 - 16) / dis2 * dx
        cmd_vel_y = ground_tar_vy + k * (dis2 - 16) / dis2 * dy
        cmd_vel_norm = math.sqrt(cmd_vel_x**2 + cmd_vel_y**2)
        if cmd_vel_norm > max_speed:
            cmd_vel_x = cmd_vel_x/cmd_vel_norm*max_speed
            cmd_vel_y = cmd_vel_y/cmd_vel_norm*max_speed

        self.__ctl_cmd.x = cmd_vel_x
        self.__ctl_cmd.y = cmd_vel_y
        self.__ctl_cmd.yaw = math.degrees(math.atan2(dy, dx))
        '''
    def prepare_aim(self):
        self.__ctl_cmd.task = CtlCMD.AIM.value
        '''
        self.__ctl_cmd.task = 4
        self.__ctl_cmd.mode = 2
        self.__ctl_cmd.z = - est_tar_z + 3 + my_UAV_home_z

        k = 0.1
        max_speed = 1
        dx = est_tar_x - my_UAV_x
        dy = est_tar_y - my_UAV_y
        dis2 = dx**2 + dy**2 + 0.01
        cmd_vel_x = est_tar_vx + k * (dis2 - 16) / dis2 * dx
        cmd_vel_y = est_tar_vy + k * (dis2 - 16) / dis2 * dy
        cmd_vel_norm = math.sqrt(cmd_vel_x**2 + cmd_vel_y**2)
        if cmd_vel_norm > max_speed:
            cmd_vel_x = cmd_vel_x/cmd_vel_norm*max_speed
            cmd_vel_y = cmd_vel_y/cmd_vel_norm*max_speed

        self.__ctl_cmd.x = cmd_vel_x
        self.__ctl_cmd.y = cmd_vel_y
        self.__ctl_cmd.yaw = my_gimbal_yaw
        '''
    def prepare_take_control(self):
        self.__ctl_cmd.task = CtlCMD.CONTROL.value
        '''
        self.__ctl_cmd.z = -my_UAV_z + my_UAV_home_z
        self.__ctl_cmd.yaw = my_gimbal_yaw
        '''
    def prepare_reject(self):
        pass

    def after_reject(self):
        #TODO: check if it is too fast to finish rejection
        self.go_home()

    def prepare_go_home(self):
        self.__ctl_cmd.task = CtlCMD.RETURN.value

    def on_enter_RETURNING(self):
        print "enter deascending"
        self.__ctl_cmd.task = CtlCMD.NONE.value
        time.sleep(0.2)
        self.returning()

      

# transition format [{action_name}, {previous_state}, {next_state}]
transitions = [
        {
            'trigger': 'take_off', 
            'source': VehicleStates.ONGROUND, 
            'dest': VehicleStates.ASCENDING,
            'prepare':'prepare_take_off',
        },
        {
            'trigger': 'ascending', 
            'source': VehicleStates.ASCENDING, 
            'dest': VehicleStates.ASCENDING,
            'conditions': 'is_ascending'
        },
        {
            'trigger': 'ascending', 
            'source': VehicleStates.ASCENDING, 
            'dest': VehicleStates.FLYING,
            'unless': 'is_ascending'
        },
        {
            'trigger': 'search', 
            'source': [
                VehicleStates.FLYING, 
                VehicleStates.AIMMING, 
                VehicleStates.CONTROLLING,
                VehicleStates.SEARCHING
            ], 
            'dest': VehicleStates.SEARCHING,
            'prepare':'prepare_search'
        },
        {
            'trigger': 'aim', 
            'source': [
                VehicleStates.FLYING, 
                VehicleStates.SEARCHING, 
                VehicleStates.CONTROLLING,
                VehicleStates.AIMMING
            ],  
            'dest': VehicleStates.AIMMING,
            'prepare':'prepare_aim'
        },
        {
            'trigger': 'take_control', 
            'source': [
                VehicleStates.FLYING, 
                VehicleStates.AIMMING, 
                VehicleStates.SEARCHING,
                VehicleStates.CONTROLLING
            ],  
            'dest':VehicleStates.CONTROLLING,
            'prepare':'prepare_take_control'
        },
        {
            'trigger': 'reject', # TODO: check reject condition
            'source': [
                VehicleStates.FLYING, 
                VehicleStates.AIMMING, 
                VehicleStates.SEARCHING,
                VehicleStates.CONTROLLING
            ],
            'dest': VehicleStates.REJECTED,
            'prepare':'prepare_reject',
            'after': 'after_reject'
        },
        {
            'trigger': 'go_home', 
            'source': [
                VehicleStates.FLYING, 
                VehicleStates.AIMMING, 
                VehicleStates.SEARCHING,
                VehicleStates.CONTROLLING
            ], 
            'dest': VehicleStates.RETURNING,
            'prepare':'prepare_go_home'
        },
        {
            'trigger': 'returning', 
            'source': VehicleStates.RETURNING,
            'dest': VehicleStates.RETURNING,
            'conditions': 'is_descending'
        },
        {
            'trigger': 'returning', 
            'source': VehicleStates.RETURNING,
            'dest': VehicleStates.ONGROUND,
            'unless': 'is_descending'
        }
    ]
