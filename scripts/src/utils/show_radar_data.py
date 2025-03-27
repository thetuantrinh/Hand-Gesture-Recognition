# -*- coding: utf-8 -*-
"""
Created on Wed Oct 16 08:39:27 2024

@author: tuant
"""
import numpy as np
import pyqtgraph as pg
from ..utils.color_map import cmap
#%%
class PLOT:
    """ Show FFT
    """
    def __init__(
            self,
            graph,
            window_data
            ):
        self.graph = graph
        self.window_data = window_data
        color = pg.ColorMap(pos=np.linspace(0.0, 1.0, 3), color=cmap())
        self.graph.setColorMap(color)
    
    def show_micro_Dopler(self):
        self.graph.setImage(
                (
                    20*np.log10(
                        np.abs(
                            self.window_data[:, 0, 0, :] + \
                       1j * self.window_data[:, 0, 1, :]
                        )
                        )
                        ),
                    levels = [65, 80]
                    )
        