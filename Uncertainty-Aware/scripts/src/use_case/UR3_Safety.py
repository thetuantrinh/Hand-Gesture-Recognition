# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 16:06:16 2024

@author: tuant
"""
#%%
class SAFETY:
    def __init__(
            self,
            TCP_position
            ): 
        """
        TCP_position [X, Y, Z, RX, RY, RZ]
        """
        self.TCP_position = TCP_position
        # self.X_upper = 183e-3 # meter
        # self.Y_upper = -350e-3 # meter
        self.Z_upper = -360e-3 # meter
        # self.X_lower = -52.94e-3 # meter
        # self.Y_lower = -330e-3 # meter
        self.Z_lower = -403.25e-3 # meter
# %%
    def limit_space_work(self):

        if abs(self.TCP_position[2]) <= abs(self.Z_upper):
            print("Z lower")
            # self.TCP_position[0] = self.X_lower
        elif abs(self.TCP_position[2]) >= abs(self.Z_lower):
            print("Z_upper")
            
            # self.TCP_position[0]   = self.X_upper
        # elif abs(self.TCP_position[2]) <= abs(self.Z_lower):
        #     print("Z_lower")
        #     # self.TCP_position[2] = self.Z_lower
        # elif abs(self.TCP_position[2]) >= abs(self.Z_upper):
        #     print("Z_upper")
        #     # self.TCP_position[2]   = self.Z_upper
#%%





