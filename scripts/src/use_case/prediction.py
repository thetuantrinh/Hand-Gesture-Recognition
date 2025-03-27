# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 15:30:37 2024

@author: tuant
"""
import os
import numpy as np
from tensorflow.keras.models import load_model
current_dir = os.path.dirname(os.path.abspath(__file__))
#%%
class PREDICTOR:
    """ Predict HGR
    """
    def __init__(
            self,
            model_path
            ):
        self.model_reg = load_model(model_path)
        self.labels = [' ', ' ', ' ',
                       'lowering',
                       'lifting',
                       'release',
                       'clamp',
                       'move to left',
                       'move to right', 
                       ' ']
        self.temp = np.ones((1, 10),
                             dtype = np.int8).squeeze()*9
        self.temp1 = np.ones((1, 2),
                              dtype = np.int8).squeeze()*9
    def prediction(
            self,
            window
            ):
        predict = np.argmax(
            self.model_reg.predict_on_batch(window),
            axis=1
            )
        self.temp = np.concatenate(
            (self.temp, predict)
            )
        self.temp = self.temp[-10:]
        self.max_cnt = np.array(
            [np.argmax(np.bincount(self.temp))]
            )
        self.temp1 = np.concatenate(
            (self.temp1, self.max_cnt)
            )
        self.temp1 = self.temp1[-2:]
        
        return self.temp1