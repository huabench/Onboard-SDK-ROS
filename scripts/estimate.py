#! /usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import math

from dji_osdk_ros.msg import iuslDetectionResult
from dji_osdk_ros.msg import iuslTarState
from modules.ekf import EKFData
from modules.measure import MeasureData

class Estimate:
    def __init__(self):
        # const
        self._net_width = 1536
        self._net_height = 864
        self._cam_f = 0.0045  # for 1536x864
        self._cam_s = 0.000004093  # for 1536x864
        self._est_UAV_real_length = 0.47

        self._R_1_c2g = np.array([[0, 0, 1],[1, 0, 0],[0, 1, 0]])
        self._intrinsic_matrix = np.array([[1099, 0, 768],[0, 1099, 432],[0, 0, 1]])   # H20T 1536x864


        rospy.init_node('estimate', anonymous=True)

        self._measure_data = MeasureData()
        self._measure_is_new = False

        self._ekf_data = EKFData()
        self._est_ekf_OK = False

        self.__rate = rospy.Rate(50)
        rospy.Subscriber('/iusl/DetectionResult', iuslDetectionResult, self.callback_receive_detection_result)
        #rospy.Subscriber('dji_osdk_ros/flight_status', UInt8, callback_flight_state)
        self.est_state_publisher = rospy.Publisher('/iusl/estimate_tar_state', iuslTarState, queue_size=5)
        self._est_state = iuslTarState()


    
    def callback_receive_detection_result(self, msg):
        self._measure_data.update(msg)
        self._measure_is_new = True

    def EKF_estimate(self):
        time_now = rospy.Time.now()
        dt = (time_now - self._measure_data.measure_time_stamp).to_sec()

        if dt > 5:  # (if there is no detect result over 5 seconds, then reinit ekf)
            self._est_ekf_OK = False

        pos_UAV = np.array([[self._measure_data.measure_UAV_x], [self._measure_data.measure_UAV_y], [self._measure_data.measure_UAV_z]])

        if self._measure_is_new:
            self._measure_is_new = False
            direction = self.calculate_direction()
            distance = self.calculate_distance()

            self._ekf_data.calculate(direction, distance, pos_UAV, self._est_ekf_OK)
            if not self._est_ekf_OK:
                self._est_ekf_OK = True
        else:
            self._ekf_data.predict(self._est_ekf_OK)

    def calculate_direction(self):
        pos_tar_p = np.array([[self._measure_data.measure_center_x],[self._measure_data.measure_center_y],[1]])
        direction_c = np.dot(np.linalg.inv(self._intrinsic_matrix), pos_tar_p)
        p_tem = math.radians(self._measure_data.measure_gim_pitch)
        r_tem = math.radians(self._measure_data.measure_gim_roll)
        y_tem = math.radians(self._measure_data.measure_gim_yaw)
        R_x = np.array([[1, 0, 0],[0, math.cos(-r_tem), math.sin(-r_tem)],[0, -math.sin(r_tem), math.cos(r_tem)]])
        R_y = np.array([[math.cos(-p_tem), 0, -math.sin(-p_tem)],[0,1,0],[math.sin(-p_tem), 0, math.cos(-p_tem)]])
        R_z = np.array([[math.cos(-y_tem), math.sin(-y_tem), 0],[-math.sin(-y_tem),math.cos(-y_tem),0],[0, 0, 1]])
        R_Mat_c2g = np.dot(np.dot(np.dot(R_z, R_y), R_x), self._R_1_c2g)
        direction_g = np.dot(R_Mat_c2g, direction_c)
        est_direction_g_bar = direction_g/np.linalg.norm(direction_g)
        return est_direction_g_bar

    def calculate_distance(self):
        measure_laser_dis = self._measure_data.measure_laser_dis
        measure_max_length = self._measure_data.measure_max_length
        dx = self._net_width/2 - self._measure_data.measure_center_x
        dy = self._net_height/2 - self._measure_data.measure_center_y
        if abs(dx) < measure_max_length/3 \
            and abs(dy) < measure_max_length/6 \
            and measure_laser_dis < 10 \
            and measure_laser_dis > 3: # (the condition of using laser)
            est_dis = measure_laser_dis
            self._est_UAV_real_length = measure_laser_dis * measure_max_length * self._cam_s / self._cam_f
        else:
            if measure_max_length < 1:  #(prevent division of zero) 
                measure_max_length = 1
            est_dis = self._est_UAV_real_length * math.sqrt(self._cam_f**2 + (dx**2 + dy**2) * (self._cam_s**2)) / (measure_max_length * self._cam_s)
        return est_dis

    def runner(self):
        while not rospy.is_shutdown():
            self.EKF_estimate()

            ekf_estimated_state = self._ekf_data.get_ekf_est_state()
            self._est_state.tar_OK = self._est_ekf_OK
            self._est_state.tar_x = ekf_estimated_state[0,0]
            self._est_state.tar_y = ekf_estimated_state[1,0]
            self._est_state.tar_z = ekf_estimated_state[2,0]
            self._est_state.tar_vx = ekf_estimated_state[3,0]
            self._est_state.tar_vy = ekf_estimated_state[4,0]

            print("estimate state", self._est_state)
            #self._est_state_publisher.publish(est_state)
                
            self.__rate.sleep()


if __name__ == "__main__":
    try:
        estimate = Estimate()
        estimate.runner()
    except rospy.ROSInterruptException:
        pass




