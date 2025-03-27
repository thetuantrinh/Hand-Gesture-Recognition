# -*- coding: utf-8 -*-
"""
Created on Thu Aug  4 09:08:56 2022

@author: tuant
"""
#%%
import codecs
import socket
import time
import struct
import numpy as np
from enum import Enum
from ..thread_fn import threading_fn

#%%
CONFIG_HEADER = '5aa5'
CONFIG_STATUS = '0000'
CONFIG_FOOTER = 'aaee'
#%%
# STATIC
MAX_PACKET_SIZE = 4096
BYTES_IN_PACKET = 1456
INT16 = 1/(2**15-1)
START_CHIRP_TX = 0
END_CHIRP_TX = 0
numLanes = 4 #number of lanes is always 4 for AWR1243BOOST ES3.0
NUM_ADC_SAMPLES = 256
num_ADC_samples_in_frame = 2048
#%%
class CMD(
        Enum
        ):
    RESET_FPGA_CMD_CODE              = '0100'
    RESET_AR_DEV_CMD_CODE            = '0200'
    CONFIG_FPGA_GEN_CMD_CODE         = '0300'
    CONFIG_EEPROM_CMD_CODE           = '0400'
    RECORD_START_CMD_CODE            = '0500'
    RECORD_STOP_CMD_CODE             = '0600'
    PLAYBACK_START_CMD_CODE          = '0700'
    PLAYBACK_STOP_CMD_CODE           = '0800'
    SYSTEM_CONNECT_CMD_CODE          = '0900'
    SYSTEM_ERROR_CMD_CODE            = '0a00'
    CONFIG_PACKET_DATA_CMD_CODE      = '0b00'
    CONFIG_DATA_MODE_AR_DEV_CMD_CODE = '0c00'
    INIT_FPGA_PLAYBACK_CMD_CODE      = '0d00'
    READ_FPGA_VERSION_CMD_CODE       = '0e00'

    def __str__(self):
        
        return str(self.value)
#%%
class DCA1000(
        object
        ):
    """Software interface to the DCA1000 EVM board via ethernet.
    """
    def __init__(
            self,
            ):
        self.log_fn = threading_fn.OutputRedirector()
        # Create configuration and data destinations
        self.cfg_dest  = ('192.168.33.180',
                          4096) #note:allow port in the firewall before run
        self.cfg_recv  = ('192.168.33.30',
                          4096) #note:allow port in the firewall before run
        self.data_recv = ('192.168.33.30',
                          4098) #note:allow port in the firewall before run
        # Create sockets
        self.config_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP
            )
        self.data_socket   = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP
            )
#%%
    def _bind_(
            self
            ):
        # Bind data socket to fpga
        self.data_socket.bind(self.data_recv)
        # Bind config socket to fpga
        self.config_socket.bind(self.cfg_recv)
        
        self.log_fn.write("Ethernet Radar port is opened!")
#%%        
    def _send_command(
            self,
            cmd,
            length='0000',
            body='',
            timeout=1
            ):
        """Helper function to send a single commmand to the FPGA
        """
        self.config_socket.settimeout(timeout)
        # Create and send message
        resp = ''
        msg = codecs.decode(''.join((CONFIG_HEADER,
                                     str(cmd),
                                     length,
                                     body,
                                     CONFIG_FOOTER)),
                            'hex')
        self.log_fn.write('Request:  ' + str(msg))
        try:
            self.config_socket.sendto(msg, self.cfg_dest)
            resp, addr = self.config_socket.recvfrom(MAX_PACKET_SIZE)            
            self.log_fn.write('Response: ' + str(resp))                     
            self.log_fn.write(str(addr) + '\n')           
        except socket.timeout as e:
            self.log_fn.write(e)
            
        return resp
#%%
    def _read_data_packet(self):
        """Helper function to read in a single ADC packet via UDP packet
        """
        try:
            data, addr = self.data_socket.recvfrom(MAX_PACKET_SIZE)
            packet_num = struct.unpack('<1l', data[:4])[0]
            # byte_count = struct.unpack('>Q', b'\x00\x00' + data[4:10][::-1])[0]
            packet_data = np.frombuffer(
                data[10:],
                dtype=np.int16
                )
        except socket.timeout as e:
            self.log_fn.write(e)
        
        return packet_num, packet_data
#%%
    def _listen_for_error(self):
        """Helper function to try and read in for an error message from the FPGA
        """
        self.config_socket.settimeout(None)
        msg = self.config_socket.recvfrom(MAX_PACKET_SIZE)
        if msg == b'5aa50a000300aaee':
            self.log_fn.write('stopped:', msg)
#%%
    def _stop_record_(
            self
            ):
        """Helper function to send the stop command to the FPGA
        """
        self.log_fn.write("Record stop \n")
        return self._send_command(
            CMD.RECORD_STOP_CMD_CODE
            )
#%%    
    def start_record(
            self
            ):
        """Helper function to send the start command to the FPGA
        """
        self.log_fn.write('Record start \n')
        self._send_command(
            CMD.RECORD_START_CMD_CODE
            )    
#%%        
    def _cfg_fpga_(
            self
            ):
        """Helper function to send the connect sys,
           configure FPGA command to the FPGA
        """
        self._send_command(
            CMD.SYSTEM_CONNECT_CMD_CODE
            )
        self.log_fn.write('System connected \n')
        self._send_command(
            CMD.RESET_FPGA_CMD_CODE
            )
        self.log_fn.write("FPGA reset")
        self.log_fn.write('FGPA Gen \n')
        self._send_command(
            CMD.CONFIG_FPGA_GEN_CMD_CODE,
            '0600',
            '01010102031E'
            )
        time.sleep(1)
        self.log_fn.write('Set package data \n')
        self._send_command(
            CMD.CONFIG_PACKET_DATA_CMD_CODE,
            '0600',
            'C005350C0000'
            ) ##1472bytes-25us

#%%    
    def close(self):
        """Closes the sockets that are used for receiving and sending data
        """
        try:
            self.data_socket.close()
            self.config_socket.close()
            self.log_fn.write("Ethernet port is closed! \n")
        except Exception as e:
            self.log_fn.write(f"Error closing socket: {e}")
#%%