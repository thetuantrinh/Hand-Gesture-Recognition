# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 16:12:06 2023

@author: tuant
"""
import time
from src import UR
import numpy as np
import logging
logging.basicConfig(
    format="%(asctime)s-%(levelname)s-%(message)s",
    level=logging.INFO
    )

#%%
class GES_POS(
        object
        ):
    def __init__(self):
        # self.ROBOT_IP = '192.168.64.128' # UR3 local IP Simulation 
        self.ROBOT_IP = '169.254.200.239'  # UR3 local IP

        logging.info("Initializing Arm Robot !")
        self.robotModel = UR.robotModel.RobotModel()
        self.robot = UR.urScriptExt.UrScriptExt(
            host=self.ROBOT_IP,
            robotModel=self.robotModel
            )
        self.robot.reset_error()
        logging.info("Initialized !")
        time.sleep(2)
        
        self.acceletion = 0.9  # Robot acceleration value
        self.velocity = 1.0    # Robot speed value
        
        self.start_pos = [55.84, #   Base
                          -73.91,  #   Shoulder
                          139.98,  #   Elbow
                          -195.87,  #   Wrist 1
                          -66.93,   #   Wrist 2
                          -203.18]    #   Wrist 3
        self.robot.set_tools(STATE = "RELEASE")
        
        self.robot.movej(
            q= np.radians(self.start_pos),
            a= self.acceletion,
            v= self.velocity
            )
        
        # starts the realtime control loop on the Universal-Robot Controller
        self.robot.init_realtime_control()  
        time.sleep(2) # just a short wait to make sure everything is initialised
#%%
    def read_ur_data(
            self,
            fps = 20,
            read_data = 'TCP Pos'
            ):
        """
        Parameters
        ----------
        fps : (int) Speed read data. The default is 20 fps.
        read_data : The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz]).
        X, Y, Z in meter, Rx, Ry, Rz in rad. The default is 'TCP Pos'. 
        
        If 'joint Pos':    
        The current actual joint angular position vector in rad : 
        [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]

        Returns
        -------
        TYPE
            DESCRIPTION.

        """
        if read_data == 'TCP Pos':    
            self.data = self.robot.get_actual_tcp_pose()
        elif read_data == 'joint Pos':
            self.data = self.robot.get_actual_joint_positions()
        # time.sleep((1/fps))
        
        return self.data
#%%
    def close(
            self
            ):
        """
        Remember to always close the robot connection,
        otherwise it is not possible to reconnect
        Returns
        -------
        None.
        Closing robot connection

        """
        self.robot.close()
