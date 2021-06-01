#! /usr/bin/env python
# coding=utf-8

import numpy as np


class EKFData:
    def __init__(self):
        self.ekf_estimated_state = np.array([[10],[0],[0], [0], [0], [0]])
        self.ekf_F = np.identity(6)
        self.ekf_F[0, 3] = 0.02
        self.ekf_F[1, 4] = 0.02
        self.ekf_F[2, 5] = 0.02
        self.ekf_Q = np.zeros([6, 6])
        self.ekf_Q[0, 0] = 0.1 
        self.ekf_Q[1, 1] = 0.1 
        self.ekf_Q[2, 2] = 0.1 
        self.ekf_Q[3, 3] = 0.1 
        self.ekf_Q[4, 4] = 0.1 
        self.ekf_Q[5, 5] = 0.1 
        self.ekf_R = np.zeros([4, 4])
        self.ekf_R[0, 0] = 0.1
        self.ekf_R[1, 1] = 0.1
        self.ekf_R[2, 2] = 0.1
        self.ekf_R[3, 3] = 0.2
        self.ekf_P = np.zeros([6, 6])
        self.ekf_P[0, 0] = 1.
        self.ekf_P[1, 1] = 1.
        self.ekf_P[2, 2] = 1.
        self.ekf_P[3, 3] = 2.
        self.ekf_P[4, 4] = 2.
        self.ekf_P[5, 5] = 2.

        self.est_UAV_real_length = 0.47 


    def calculate(self, direction, distance, pos_UAV, est_ekf_OK):
        # -------- measure state ------------
        ekf_mear_state = np.vstack((direction, distance))
        # -------- EKF process -----------
        self.ekf_R[3, 3] = 0.2 * distance
        H1 = (np.identity(3) - np.dot(direction, np.transpose(direction))) / (distance + 0.01)
        H1 = np.vstack((H1, np.transpose(direction)))
        self.ekf_H = np.hstack((H1, np.zeros([4, 3])))
        if not est_ekf_OK: 
            self.ekf_estimated_state = np.vstack((pos_UAV + direction * distance, np.array([[0], [0], [0]])))
        else:
            self.ekf_estimated_state = np.dot(self.ekf_F, self.ekf_estimated_state)
            self.ekf_P = np.dot(np.dot(self.ekf_F, self.ekf_P), np.transpose(self.ekf_F)) + self.ekf_Q
            g_hat = self.ekf_estimated_state[0:3] - pos_UAV
            distance_hat = np.linalg.norm(g_hat)
            g_bar_hat = g_hat/distance_hat
            mear_state_hat = np.vstack((g_bar_hat, distance_hat))
            K = np.dot(np.dot(self.ekf_P, np.transpose(self.ekf_H)), np.linalg.inv(np.dot(np.dot(self.ekf_H, self.ekf_P), np.transpose(self.ekf_H)) + self.ekf_R))
            self.ekf_estimated_state = self.ekf_estimated_state + np.dot(K, ekf_mear_state - mear_state_hat)
            n_tem, _ = K.shape
            self.ekf_P = np.dot(np.identity(n_tem) - np.dot(K, self.ekf_H), self.ekf_P)

    def predict(self, est_ekf_OK):
        if est_ekf_OK:
            self.ekf_estimated_state = np.dot(self.ekf_F, self.ekf_estimated_state)

    def get_ekf_est_state(self):
        return self.ekf_estimated_state