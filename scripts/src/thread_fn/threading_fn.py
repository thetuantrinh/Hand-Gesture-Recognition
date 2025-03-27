# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 14:07:31 2024

@author: tuant
"""
from threading import Thread
from PyQt5.QtCore import (QObject, pyqtSignal,
                          QRunnable, pyqtSlot)
from src.radar import DCA1000EVM_backend
from src.UR import UR3_GESTURE as ur3
#%%
class OutputRedirector(QObject):
    new_output = pyqtSignal(str)

    def write(self, message):
        self.new_output.emit(message)

    def flush(self):
        pass
#%%
class WorkerSignals(
        QObject
        ):
    '''
    Defines the signals available from a running worker thread.
    '''
    bin_data_signal = pyqtSignal(object)
    robot_signal    = pyqtSignal(object)
    
#%%
class Thread_Prediction(
        Thread
        ):
    '''
    Prediction thread to run the TF model
    '''
    def __init__(
            self,
            group=None,
            target=None,
            name=None,
            args=(),
            kwargs={},
            Verbose=None
            ):
        Thread.__init__(
            self,
            group,
            target,
            name,
            args,
            kwargs
            )
        self._return = None

    def run(
            self
            ):
        if self._target is not None:
            self._return = self._target(
                *self._args,
                **self._kwargs
                )
    def join(
            self,
            *args
            ):
        Thread.join(
            self,
            *args
            )
        
        return self._return

#%%
class Radar(
        QRunnable
        ):
    '''
    Worker Radar thread to read raw data
    '''
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            Radar,
            self
            ).__init__(*args, **kwargs)
        self.read_raw_data = DCA1000EVM_backend.DCA1000()
        self.signals = WorkerSignals()

    @pyqtSlot(object)
    def run(
            self
            ):
        '''
        Initialise the runner function.
        '''
        while True:
            self.num, self.adcData = \
                self.read_raw_data._read_data_packet()
            self.signals.bin_data_signal.emit(
                self.adcData
                ) 
            
#%%            
class UR3(
        QRunnable
        ):
    '''
    Worker Robot thread to read data from Universal Robots
    '''
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            UR3,
            self
            ).__init__(*args, **kwargs)
        self.control_robot = ur3.GES_POS()
        self.signals = WorkerSignals()

    @pyqtSlot(object)
    def run(
            self
            ):
        '''
        Initialise the runner function.
        '''
        while True:
            self.tcp_pos_data = self.control_robot.read_ur_data(
                fps = 20,
                read_data = 'TCP Pos'
                )
            self.joint_pos_data = self.control_robot.read_ur_data(
                fps = 20,
                read_data = 'joint Pos'
                )
            self.signals.robot_signal.emit(
                [self.tcp_pos_data,
                 self.joint_pos_data]
                ) 
#%%