# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 20:47:05 2024

@author: tuant
"""
import numpy as np

def SHOW_UR3_TCP_POS(X, Y, Z, RX, RY, RZ,
                     self_X, self_Y, self_Z,
                     self_RX, self_RY, self_RZ):

    x_meter = str(np.round(X*1000, 2))
    y_meter = str(np.round(Y*1000, 2))
    z_meter = str(np.round(Z*1000, 2))
    
    RX_degree = str(np.round(RX, 2))
    RY_degree = str(np.round(RY, 2))
    RZ_degree = str(np.round(RZ, 2))

    self_X.setText(x_meter); self_RX.setText(RX_degree)
    self_Y.setText(y_meter); self_RY.setText(RY_degree)
    self_Z.setText(z_meter); self_RZ.setText(RZ_degree)
    
def SHOW_UR3_JOINT_POS(B, S, E, W1, W2, W3,
                       self_B, self_S, self_E,
                       self_W1, self_W2, self_W3):
    
    B_degree  = str(np.round(np.degrees(B), 2))
    S_degree  = str(np.round(np.degrees(S), 2))
    E_degree  = str(np.round(np.degrees(E), 2))
    W1_degree = str(np.round(np.degrees(W1), 2))
    W2_degree = str(np.round(np.degrees(W2), 2))
    W3_degree = str(np.round(np.degrees(W3), 2))

    self_B.setText(B_degree); self_W1.setText(W1_degree)
    self_S.setText(S_degree); self_W2.setText(W2_degree)
    self_E.setText(E_degree); self_W3.setText(W3_degree)