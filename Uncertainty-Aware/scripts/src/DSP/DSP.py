# -*- coding: utf-8 -*-
"""
Created on Fri Dec 16 15:42:17 2022

@author: tuant
"""
import numpy as np
from numpy.fft import fftshift, fft
#%%
class _FFT_:
    """ Performed Discrete Fast Fourier Transform
    """
    def __init__(
            self,
            Nc: int,    ### No. of Chirp Loops
            Ns: int     ### Number ADC samples
            ):           
        self.adc_bit = 16
        self.Ns = Ns
        self.Nc = Nc
        self.Nt = 3                          ## Number of transmit antennas
        self.Nr = 4                          ## Number of received antennas
        self.vir_an = self.Nt * self.Nr 
        self.num_angle_bins = Nc
        self.win1D_Ns = np.blackman(self.Ns) ## Recommend using Blackman window
        self.win1D_Nc = np.hanning(self.Nc)  ## Recommend using Hanning window
        
#%% RANGE PROFILE
    def fft_1D(
            self,
            adc_data
            ):
        """
        Perform 1D FFT on complex-format ADC data.
        Args:
            adc_data (ndarray): [Nr, Nc, Ns] Format raw data in frame.
        Returns:
            [Nr, Nc, Ns] (ndarray): Range bin data in dB
        """        
        windowing2D = np.multiply(adc_data, self.win1D_Ns)
        out_fft1D   = fft(windowing2D, axis = 2)
        # fft_shift = np.abs(fftshift(out_fft1D, axes=(2,)))
        # out_dB = 20*np.log10(fft_shift)
        
        return out_fft1D
    
#%% DOPPLER FFT RANGE-VELOCITY PROFILE
    def fft_2D(
            self,
            adc_data,
            win_1D_fuc = True,
            win_2D_fuc = True
            ):
        """
        Perform 2D FFT on complex-format 1D FFT .

        Parameters
        ----------
        adc_data : (ndarray): [Nr, Nc, Ns] Format raw data in frame.
        win_1D_fuc : bool, The default is False. The window function samples
        win_2D_fuc : bool, The default is False. The window function chirps

        Returns
        -------
        TYPE
            output FFT-2D [Nr, IQ, Ns, Nc]

        """
        #% Fast-time Processing (across samples)
        windowing1D = np.multiply(adc_data, self.win1D_Ns)
        if win_1D_fuc:
            out_fft1D   = fft(windowing1D, axis = 2)
        else:
            out_fft1D   = fft(adc_data, axis = 2)
        #% Remove zero-Doppler (back-ground data)
        out_fft1D = self.MTI(out_fft1D)    
        #% Slow-time Processing (across chirps)
        windowing2D = np.multiply(out_fft1D.transpose(0, 2, 1),
                                  self.win1D_Nc)
        if win_2D_fuc:
            out_fft2D   = fft(windowing2D, axis = 2)
        else:
            out_fft2D   = fft(out_fft1D, axis = 1).transpose(0, 2, 1)
        fft_shift = fftshift(out_fft2D) + 1e-9

        return np.array([[fft_shift[0].real, fft_shift[0].imag],
                         [fft_shift[1].real, fft_shift[1].imag],
                         [fft_shift[2].real, fft_shift[2].imag],
                         [fft_shift[3].real, fft_shift[3].imag]],
                        dtype = np.float32)
    
#%% Static clutter removal
    def MTI(
            self,
            data
            ):
        """
        Once the active chirp time of the frame is complete, the interframe
        processing can begin, starting with static clutter removal.
        1D FFT data is averaged across all chirps for a single Virtual Rx antenna.
        This average is then subtracted from each chirp from the Virtual Rx antenna.
        This cleanly removes the static information from the signal,
        leaving only the signals returned from moving objects.        

        Parameters
        ----------
        data : (complex64) FFT - 1D output.

        Returns
        -------
        None.
        """
        Xnr = (1/(self.Nc)) * np.sum(data, axis = 1, keepdims=True)
        Xncr= data - Xnr
        
        return Xncr
#%%
    def pre_processing(
            self, 
            adcData,
            fft = False
            ):
        adcData = np.reshape(
            adcData,
            (8, self.Nc * self.Ns),
            order = 'F'
            )
        adcData = adcData[[0, 1, 2, 3], :] + \
             1j * adcData[[4, 5, 6, 7], :]
        adcData = np.array(
            [
                 np.reshape(
                 adcData[0], 
                 (self.Nc, self.Ns)
                 ),
             np.reshape(
                 adcData[1],
                 (self.Nc, self.Ns)
                 ),
             np.reshape(
                 adcData[2],
                 (self.Nc, self.Ns)
                 ),
             np.reshape(
                 adcData[3],
                 (self.Nc, self.Ns)
                 )
             ]
            )
        
        if fft:
            return self.fft_2D(adcData)
        
        return adcData         