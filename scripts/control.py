#! /usr/bin/env python
# coding=utf-8

import rospy
from dji_osdk_ros.srv import SetGoHomeAltitude
from dji_osdk_ros.srv import ObtainControlAuthority

from transitions.extensions import HierarchicalMachine as Machine
from modules.common_type import VehicleStates, GroundCMD
from modules.vehicle_model import Model, transitions

#from transitions.extensions import GraphMachine as Machine

class Control:
    def __init__(self):
        self.__vehicle = Model()
        self.__machine = Machine(
            model=self.__vehicle,
            states=VehicleStates, 
            transitions=transitions, 
            initial=VehicleStates.ONGROUND, 
            ignore_invalid_triggers=True,
            #send_event=True,
            before_state_change='send_ctl_cmd'
            #use_pygraphviz=False
        )
        rospy.init_node('control', anonymous=True)
        self.__rate = rospy.Rate(100)

    def test(self):
        #TODO: searching is not in machine
        #self.__vehicle.to_SEARCHING()
        print(self.__vehicle.state)
        self.__vehicle.take_off()
        print(self.__vehicle.state)
        self.__vehicle.search()
        print(self.__vehicle.state)

    def runner(self):
        g_cmd_list = [
            GroundCMD.TAKEOFF, 
            GroundCMD.NONE, 
            GroundCMD.NONE, 
            GroundCMD.SEARCH,
            GroundCMD.NONE, 
            GroundCMD.NONE, 
            GroundCMD.NONE, 
            GroundCMD.NONE, 
            GroundCMD.RETURN
            ]
        ind = 0
        while not rospy.is_shutdown():
            # convert 0...4 to enum
            # g_cmd = GroundCMD(self.__vehicle.get_ground_cmd())
            g_cmd = g_cmd_list[ind]
            ind = min(ind + 1, 8 )
            if g_cmd == GroundCMD.TAKEOFF:
                self.__vehicle.take_off()
            elif g_cmd == GroundCMD.SEARCH:
                self.__vehicle.search() #TODO: inner loop
            elif g_cmd == GroundCMD.AIM:
                self.__vehicle.aim() #TODO: inner loop
            elif g_cmd == GroundCMD.CONTROL:
                self.__vehicle.take_control()
            elif g_cmd == GroundCMD.REJECT:
                self.__vehicle.reject()
            elif g_cmd == GroundCMD.RETURN:
                self.__vehicle.go_home()
            elif g_cmd == GroundCMD.NONE:
                if self.__vehicle.state == VehicleStates.SEARCHING:
                    self.__vehicle.search()
                elif self.__vehicle.state == VehicleStates.AIMMING:
                    self.__vehicle.aim()

            else:
                print("invalid instruction")
            self.__rate.sleep()
            
            

if __name__ == "__main__":
    c = Control()
    #c.test()
    c.runner()
