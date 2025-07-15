# -*- coding: utf-8 -*-
"""
Created on Thu Oct 10 14:13:42 2024

@author: tuant
"""
import time

class AUTO:
    """Auto
    TCP_pos [X, Y, Z, RX, RY, RZ]
    """
    def __init__(
            self,
            ur3,
            TCP_position
            ): 
        self.ur3 = ur3
        self.dis = 30e-3 # meter
        self.TCP_position = TCP_position

    def thread_control(self,
                       HGR_SIG):
        
        if HGR_SIG == 3:
                    self.ur3.control_robot.robot.set_realtime_pose([
                        self.TCP_position[0],
                        self.TCP_position[1],
                        self.TCP_position[2]-self.dis,
                        self.TCP_position[3],
                        self.TCP_position[4],
                        self.TCP_position[5]]
                        )
                    # time.sleep(1)
                
        elif HGR_SIG == 4:
                    self.ur3.control_robot.robot.set_realtime_pose([
                        self.TCP_position[0],
                        self.TCP_position[1],
                        self.TCP_position[2]+self.dis,
                        self.TCP_position[3],
                        self.TCP_position[4],
                        self.TCP_position[5]]
                        )
                    # time.sleep(1)

        elif HGR_SIG == 5:
                self.ur3.control_robot.robot.set_tools(STATE = "RELEASE")
                # time.sleep(1)
                
        elif HGR_SIG == 6:
                self.ur3.control_robot.robot.set_tools(STATE = "CLAMP")
                # time.sleep(1)
                    
        elif HGR_SIG == 7:
                    self.ur3.control_robot.robot.set_realtime_pose([
                        self.TCP_position[0]+self.dis,
                        self.TCP_position[1],
                        self.TCP_position[2],
                        self.TCP_position[3],
                        self.TCP_position[4],
                        self.TCP_position[5]]
                        )
                    # time.sleep(1)
                    
        elif HGR_SIG == 8:
                    self.ur3.control_robot.robot.set_realtime_pose([
                        self.TCP_position[0]-self.dis,
                        self.TCP_position[1],
                        self.TCP_position[2],
                        self.TCP_position[3],
                        self.TCP_position[4],
                        self.TCP_position[5]]
                        )   
                    # time.sleep(1)
#%%
class MANUAL:
    """Manual
    TCP_pos [X, Y, Z, RX, RY, RZ]
    """
    def __init__(
            self,
            ur3,
            TCP_position
            ): 
        self.ur3 = ur3
        self.dis = 30e-3 # meter
        self.TCP_position = TCP_position
        
    def up_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0],
                    self.TCP_position[1],
                    self.TCP_position[2]+self.dis,
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )
    def down_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0],
                    self.TCP_position[1],
                    self.TCP_position[2]-self.dis,
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )
        
    def in_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0],
                    self.TCP_position[1]-self.dis,
                    self.TCP_position[2],
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )

    def out_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0],
                    self.TCP_position[1]+self.dis,
                    self.TCP_position[2],
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )
        
    def left_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0]-self.dis,
                    self.TCP_position[1],
                    self.TCP_position[2],
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )
                    
    def right_state(self):
                self.ur3.control_robot.robot.set_realtime_pose([
                    self.TCP_position[0]+self.dis,
                    self.TCP_position[1],
                    self.TCP_position[2],
                    self.TCP_position[3],
                    self.TCP_position[4],
                    self.TCP_position[5]]
                    )
                
    def clamp_state(self):
        self.ur3.control_robot.robot.set_tools(STATE = "CLAMP")
                
    def release_state(self):
        self.ur3.control_robot.robot.set_tools(STATE = "RELEASE")

#%%