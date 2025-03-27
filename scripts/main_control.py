import time
import sys, os
import threading
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThreadPool, QDateTime
from PyQt5.QtWidgets import QFileDialog
from src.DSP import DSP
from src.thread_fn.threading_fn import (WorkerSignals,
                                        Thread_Prediction,
                                        Radar,
                                        UR3)
from src.UI import _UI_
from src.use_case.UR3_CTRL import MANUAL, AUTO
from src.use_case.prediction import PREDICTOR
from src.utils.show_ur3_data import (SHOW_UR3_TCP_POS,
                                     SHOW_UR3_JOINT_POS)
from src.utils.show_radar_data import PLOT
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

#%%       
class MainWindow(
        _UI_.Ui_MainWindow
        ):
    def __init__(
            self,
            *args,
            **kwargs
            ):
        super(
            MainWindow,
            self
            ).__init__(*args, **kwargs)
        self.mainWindow = QtWidgets.QMainWindow()
        self.setupUi(self.mainWindow)
        
        self.signals = WorkerSignals()
        self.radar = Radar()       
        self.ur3 = UR3()
        
        # self.output_redirector = OutputRedirector()
        self.radar.read_raw_data.log_fn.new_output.connect(self.append_output)
        sys.stdout = self.radar.read_raw_data.log_fn
        sys.stderr = self.radar.read_raw_data.log_fn
        self.log_fn = self.radar.read_raw_data.log_fn
        
        self.btn_start_radar.clicked.connect(self.thread_read_radar)
        self.btn_stop_radar.clicked.connect(self.stop_close_radar)
        self.btn_start_UR3.clicked.connect(self.thread_read_ur3)
        self.btn_stop_UR3.clicked.connect(self.stop_ur3)
        self.btn_RESTART_RADAR.clicked.connect(self.reset_cfg_radar)
        self.btn_reset_UR3.clicked.connect(self.reset_cfg_ur3)
        self.btn_file.clicked.connect(self.browseFile)
        
        self.UP.clicked.connect(self.up)
        # self.DOWN.clicked.connect(self.down)
        self.LEFT.clicked.connect(self.left)
        self.RIGHT.clicked.connect(self.right)
        self.IN.clicked.connect(self.in_)
        self.OUT.clicked.connect(self.out)
        
        self.option_clamp.toggled.connect(self.clamp)
        self.option_release.toggled.connect(self.release)
        
        self.UR3_TCP = [-0.0366, -0.3664,  0.1467, 
                        2.8804, -0.5615, -1.0891]
        
        self.idx_ges = 9
        self.labels = [' ', ' ', ' ',
                       'lowering the workpiece',
                       'lifting the workpiece',
                       'release the workpiece',
                       'clamp the workpiece',
                       'move to left',
                       'move to right', 
                       ' ']
        
        self.threadpool = QThreadPool.globalInstance()
        self.log_fn.write(
            f'Multithreading with maximum {self.threadpool.maxThreadCount()} threads'
              )
        
        self.HRG_pred = PREDICTOR("C:/Users/ADMIN/OneDrive - Phenikaa Univesity/Desktop/real-time/models/model_2.h5")
        
        self.Line = np.array([], dtype = np.int16)
        self.Ns = 64
        self.Nc = 128
        self.Nr = 4
        self.IQ = 2
        self.num_Ns = self.Ns * self.Nr * self.IQ * self.Nc
        self.micro = np.ones((40, 4, 2, 40))
        self.dsp = DSP._FFT_(self.Nc, self.Ns)
        
        self.btn_a.clicked.connect(self.toggleExpand)
        self.expanded = False
        self.originalSize = self.mainWindow.size()
#%%       
    def browseFile(self):
        file_dialog = QFileDialog.getOpenFileName(
            self.mainWindow,
            'Open File',
            '',
            'All Files (*);;Text Files (*.txt)')
        if file_dialog[0]:
            self.selected_file_path = file_dialog[0]
            print(f'Load KERAS model from path: {self.selected_file_path}')
            self.path_model.setText(f"{self.selected_file_path}")
#%%
    def toggleExpand(self):
        if self.expanded:
            self.mainWindow.resize(self.originalSize)
            self.expanded = False
        else:
            self.mainWindow.resize(int(self.mainWindow.width()*1.4),
                                   self.mainWindow.height())
            self.expanded = True
#%%        
    def connect_rs_cfg(self): ### DCA1000EVM Init ######
        self.radar.read_raw_data._bind_()
        self.radar.read_raw_data._cfg_fpga_()  
        
    def stop_close_radar(self):
        self.radar.read_raw_data._stop_record_()
        time.sleep(1)
        self.radar.read_raw_data.close()
        time.sleep(1)
        self.btn_start_radar.setEnabled(True)
        self.btn_RESTART_RADAR.setEnabled(True)
        
    def read_radar(
            self,
            UDP_packet
            ):   
        self.Line = np.concatenate((self.Line, UDP_packet))
        if len(self.Line)>=self.num_Ns:
            frame = self.Line[:self.num_Ns]
            self.Line = self.Line[self.num_Ns:]
            data = self.dsp.pre_processing(np.array([frame]), fft = True)
            fft_cube = np.expand_dims(data[:, :, 33, 44:84], axis = 0)
            self.micro = np.concatenate((self.micro, fft_cube), axis = 0)
            self.micro = self.micro[-40:, :, :, :]
            PLOT(self.graph, self.micro).show_micro_Dopler()
            win_ges = self.micro[20:, :, :, :].transpose(0, 3, 1, 2)
            if self.pred.isChecked() == True:
                    # print("Start Hand Gesture Prediction")
                    th_pred = Thread_Prediction(
                        target=self.HRG_pred.prediction,
                        args=(win_ges.reshape(1, 20, 40, 8),)
                        )
                    th_pred.start()
                    
                    if th_pred.join()[1] != th_pred.join()[0]:
                        self.idx_ges = th_pred.join()[1]
                        text = self.labels[int(self.idx_ges)]
                        self.gesture.setText(text)
                    
                    if self.ctrl_ur3.isChecked() == True:      
                        th_ctrl = threading.Thread(
                            target=self.hgr_ur3_control,
                            args = (self.idx_ges,)
                            )
                        th_ctrl.start()
                        th_ctrl.join()
#%%
    def thread_process_ur3(self,
                           UR3_DATA):
        """ UR3_DATA: [X, Y, Z, RX, RY, RZ,
                      B , S, E, W1, W2, W3]
        """
        TCP = UR3_DATA[0]; JOINT = UR3_DATA[1]
        self.UR3_TCP = TCP
        SHOW_UR3_TCP_POS(
            TCP[0], TCP[1], TCP[2],
            TCP[3], TCP[4], TCP[5],
            self.X,  self.Y,  self.Z,
            self.RX, self.RY, self.RZ)
        SHOW_UR3_JOINT_POS(
            JOINT[0], JOINT[1], JOINT[2],
            JOINT[3], JOINT[4], JOINT[5],
            self.base,    self.shoulder, self.elbow,
            self.wrist_4, self.wrist_5,  self.wrist_6)
        
    def hgr_ur3_control(self, HGR_SIG):
        AUTO(self.ur3,
             self.UR3_TCP).thread_control(HGR_SIG)   
    
    def up(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).up_state()
    # def down(self):
    #         MANUAL(self.ur3,
    #                self.UR3_TCP).down_state()
    def in_(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).in_state()
    def out(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).out_state()
    def left(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).left_state()
    def right(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).right_state()
    def clamp(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).clamp_state()
    def release(self):
            MANUAL(self.ur3,
                   self.UR3_TCP).release_state()
#%%        
    def stop_ur3(self):
        self.ur3.control_robot.close()
        self.btn_start_UR3.setEnabled(True)
        self.btn_reset_UR3.setEnabled(True)
#%%
    def thread_read_radar(
            self
            ):
        self.connect_rs_cfg()
        self.radar.read_raw_data.start_record()
        self.btn_start_radar.setEnabled(False)
        self.radar.signals.bin_data_signal.connect(self.read_radar)
        self.threadpool.start(self.radar)

    def thread_read_ur3(
            self
            ):
        self.btn_start_UR3.setEnabled(False)
        self.ur3.signals.robot_signal.connect(self.thread_process_ur3)
        self.threadpool.start(self.ur3)
        
#%%
    def reset_cfg_radar(self):
        self.btn_RESTART_RADAR.setEnabled(False)
        print(f'Set system IP Address: {self.sys_IP_ADD_0.text()}.{self.sys_IP_ADD_1.text()}.{self.sys_IP_ADD_2.text()}.{self.sys_IP_ADD_3.text()}')
        print(f'Set FPGA IP Address: {self.FPGA_IP_ADD_0.text()}.{self.FPGA_IP_ADD_1.text()}.{self.FPGA_IP_ADD_2.text()}.{self.FPGA_IP_ADD_3.text()}')
        print(f'Set Configure Port: {self.CFG_PORT.value()}')
        print(f'Set Record Data Port: {self.REG_PORT.value()}')
        print(f'Set Packet Delay: {self.delay_pkg.value()}')
    
    def reset_cfg_ur3(self):
        self.btn_reset_UR3.setEnabled(False)
        print(f'Set UR3 IP Address: {self.UR3_IP_ADD_0.text()}.{self.UR3_IP_ADD_1.text()}.{self.UR3_IP_ADD_2.text()}.{self.UR3_IP_ADD_3.text()}')
        print(f'Set RTDE Port: {self.RTDE_PORT.value()}')
        
#%%
    def append_output(self, message):
        current_time = QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss')
        self.text_edit.append(current_time +'-:-  ' + message)
        self.text_edit.verticalScrollBar().setValue(
            self.text_edit.verticalScrollBar().maximum())
#%%
app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.mainWindow.show()
sys.exit(app.exec())
#%%