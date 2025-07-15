from src import UR

class RobotModel(object):

    def __init__(self,
                 log_path="ur_log/",
                 log_config_path=None):
        '''
        Constructor see class description for more info.
        '''
        logger = UR.dataLogging.DataLogging(path=log_path, config=log_config_path)
        name = logger.AddEventLogging(__name__)
        self.__logger = logger.__dict__[name]
        self.__logger.info('Init done')

        #Universal Robot Model content
        self.password = None
        self.ipAddress = None

        self.dataDir = {'timestamp':None,
                         'target_q':None,
                         'target_q':None,
                         'target_qd':None,
                         'target_qdd':None,
                         'target_current':None,
                         'target_moment':None,
                         'actual_q':None,
                         'actual_qd':None,
                         'actual_current':None,
                         'joint_control_output':None,
                         'actual_TCP_pose':None,
                         'actual_TCP_speed':None,
                         'actual_TCP_force':None,
                         'target_TCP_pose':None,
                         'target_TCP_speed':None,
                         'actual_digital_input_bits':None,
                         'joint_temperatures':None,
                         'actual_execution_time':None,
                         'robot_mode':None,
                         'joint_mode':None,
                         'safety_mode':None,
                         'actual_tool_accelerometer':None,
                         'speed_scaling':None,
                         'target_speed_fraction':None,
                         'actual_momentum':None,
                         'actual_main_voltage':None,
                         'actual_robot_voltage':None,
                         'actual_robot_current':None,
                         'actual_joint_voltage':None,
                         'actual_digital_output_bits':None,
                         'runtime_state':None,
                         'robot_status_bits':None,
                         'safety_status_bits':None,
                         'analog_io_types':None,
                         'standard_analog_input0':None,
                         'standard_analog_input1':None,
                         'standard_analog_output0':None,
                         'standard_analog_output1':None,
                         'io_current':None,
                         'euromap67_input_bits':None,
                         'euromap67_output_bits':None,
                         'euromap67_24V_voltage':None,
                         'euromap67_24V_current':None,
                         'tool_mode':None,
                         'tool_analog_input_types':None,
                         'tool_analog_input0':None,
                         'tool_analog_input1':None,
                         'tool_output_voltage':None,
                         'tool_output_current':None,
                         'tcp_force_scalar':None,
                         'output_bit_registers0_to_31':None,
                         'output_bit_registers32_to_63':None,
                         'output_int_register_0':None,
                         'output_int_register_1':None,
                         'output_int_register_2':None,
                         'output_int_register_3':None,
                         'output_int_register_4':None,
                         'output_int_register_5':None,
                         'output_int_register_6':None,
                         'output_int_register_7':None,
                         'output_int_register_8':None,
                         'output_int_register_9':None,
                         'output_int_register_10':None,
                         'output_int_register_11':None,
                         'output_int_register_12':None,
                         'output_int_register_13':None,
                         'output_int_register_14':None,
                         'output_int_register_15':None,
                         'output_int_register_16':None,
                         'output_int_register_17':None,
                         'output_int_register_18':None,
                         'output_int_register_19':None,
                         'output_int_register_20':None,
                         'output_int_register_21':None,
                         'output_int_register_22':None,
                         'output_int_register_23':None,
                         'output_double_register_0':None,
                         'output_double_register_1':None,
                         'output_double_register_2':None,
                         'output_double_register_3':None,
                         'output_double_register_4':None,
                         'output_double_register_5':None,
                         'output_double_register_6':None,
                         'output_double_register_7':None,
                         'output_double_register_8':None,
                         'output_double_register_9':None,
                         'output_double_register_10':None,
                         'output_double_register_11':None,
                         'output_double_register_12':None,
                         'output_double_register_13':None,
                         'output_double_register_14':None,
                         'output_double_register_15':None,
                         'output_double_register_16':None,
                         'output_double_register_17':None,
                         'output_double_register_18':None,
                         'output_double_register_19':None,
                         'output_double_register_20':None,
                         'output_double_register_21':None,
                         'output_double_register_22':None,
                         'output_double_register_23':None,
                         'urPlus_force_torque_sensor':None,
                         'urPlus_totalMovedVerticalDistance':None
                         }

        self.rtcConnectionState = None
        self.rtcProgramRunning = False
        self.rtcProgramExecutionError = False
        self.stopRunningFlag = False
        self.forceRemoteActiveFlag = False
        self.realtimeControlFlag = False
        
        # UR plus content
        self.hasForceTorqueSensor = False
        self.forceTourqe = None
#%%
    def RobotTimestamp(self):
        return self.dataDir['timestamp']
    def RuntimeState(self): 
        return self.rtcProgramRunning
    def StopRunningFlag(self): 
        return self.stopRunningFlag
#%%    
    def DigitalInputbits(self,n):
        if n>=0 & n<8:
            n = pow(2,n)
            return n&self.dataDir['actual_digital_input_bits']==n
        else:
            return None
#%%
    def ConfigurableInputBits(self,n):
        if n>=8 & n<16:
            n = pow(2,n+8)
            return n&self.dataDir['actual_digital_input_bits']==n
        else:
            return None
#%%
    def DigitalOutputBits(self,n):
        if n>=0 & n<8:
            n = pow(2,n)
            return n&self.dataDir['actual_digital_output_bits']==n
        else:
            return None
#%%
    def ConfigurableOutputBits(self,n):
        if n>=8 & n<16:
            n = pow(2,n+8)
            return n&self.dataDir['actual_digital_output_bits']==n
        else:
            return None
#%%
    def ActualTCPPose(self):
        return self.dataDir['actual_TCP_pose']
#%%   
    def ActualQ(self):
        return self.dataDir['actual_q']
#%%
    def ActualTCPSpeed(self):
        return self.dataDir["actual_TCP_speed"]
#%%
    def ActualTCPForce(self):
        return self.dataDir['actual_TCP_force']
#%%
    def StandardAnalogInput(self,n):
        if n == 0:
            return self.dataDir['standard_analog_input0']
        elif n == 1:
            return self.dataDir['standard_analog_input1']
        else:
            raise KeyError('Index out of range')
#%%
    def RobotStatus(self):
        '''
        SafetyStatusBit class defined in the bottom of this file
        '''
        result = RobotStatusBit()
        result.PowerOn            =  1&self.dataDir['robot_status_bits']==1
        result.ProgramRunning     =  2&self.dataDir['robot_status_bits']==2
        result.TeachButtonPressed =  4&self.dataDir['robot_status_bits']==4
        result.PowerButtonPressed =  8&self.dataDir['robot_status_bits']==8
        
        return result
#%%
    def SafetyStatus(self):
        '''
        SafetyStatusBit class defined in the bottom of this file
        '''
        result = SafetyStatusBit()
        result.NormalMode             =     1&self.dataDir['safety_status_bits']==1
        result.ReducedMode            =     2&self.dataDir['safety_status_bits']==2
        result.ProtectiveStopped      =     4&self.dataDir['safety_status_bits']==4
        result.RecoveryMode           =     8&self.dataDir['safety_status_bits']==8
        result.SafeguardStopped       =    16&self.dataDir['safety_status_bits']==16
        result.SystemEmergencyStopped =    32&self.dataDir['safety_status_bits']==32
        result.RobotEmergencyStopped  =    64&self.dataDir['safety_status_bits']==64
        result.EmergencyStopped       =   128&self.dataDir['safety_status_bits']==128
        result.Violation              =   256&self.dataDir['safety_status_bits']==256
        result.Fault                  =   512&self.dataDir['safety_status_bits']==512
        result.StoppedDueToSafety     =  1024&self.dataDir['safety_status_bits']==1024
        
        return result
#%%
    def TcpForceScalar(self): 
        return self.dataDir['tcp_force_scalar']
#%%
    def OutputBitRegister(self):
        result = [None]*64
        for ii in range(64):
            if ii<32 and self.dataDir['output_bit_registers0_to_31'] is not None:
                result[ii] = 2**(ii)&self.dataDir['output_bit_registers0_to_31']==2**(ii)
            elif ii>31 and self.dataDir['output_bit_registers32_to_63'] is not None:
                result[ii] = 2**(ii-32)&self.dataDir['output_bit_registers32_to_63']==2**(ii-32)
        return result
#%%
    def OutputDoubleRegister(self, n):
        return self.dataDir["output_double_register_" + str(n)]
#%%
class RobotStatusBit(object):
    PowerOn = None
    ProgramRunning = None
    TeachButtonPressed = None
    PowerButtonPressed = None
#%%
class SafetyStatusBit(object):
    NormalMode = None
    ReducedMode = None
    ProtectiveStopped = None
    RecoveryMode = None
    SafeguardStopped = None
    SystemEmergencyStopped = None
    RobotEmergencyStopped = None
    EmergencyStopped = None
    Violation = None
    Fault = None
    StoppedDueToSafety = None
