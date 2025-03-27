from src.UR.urScript import UrScript
from src.UR import dataLogging

class UrScriptExt(UrScript):

    def __init__(self,
                 host,
                 robotModel,
                 hasForceTorque=False, 
                 conf_filename=None):
        if host is None:  # Only for enable code completion
            return
        super(UrScriptExt, self).__init__(host,
                                          robotModel, 
                                          hasForceTorque,
                                          conf_filename)
        logger = dataLogging.DataLogging()
        name = logger.AddEventLogging(__name__, log2Consol=False)
        self.__logger = logger.__dict__[name]
        self.print_actual_tcp_pose()
        self.print_actual_joint_positions()
        self.__logger.info('Init done')

    def close(self):
        # self.print_actual_tcp_pose()
        # self.print_actual_joint_positions()
        self.robotConnector.close()

    def reset_error(self):
        '''
        Check if the UR controller is powered on and ready to run.
        If controller isn't power on it will be power up.
        If there is a safety error, it will be tried rest it once.

        Return Value:
        state (boolean): True of power is on and no safety errors active.
        '''

        if not self.robotConnector.RobotModel.RobotStatus().PowerOn:
            # self.robotConnector.DashboardClient.PowerOn()
            self.robotConnector.DashboardClient.ur_power_on()
            self.robotConnector.DashboardClient.wait_dbs()
            # self.robotConnector.DashboardClient.BrakeRelease()
            self.robotConnector.DashboardClient.ur_brake_release()
            self.robotConnector.DashboardClient.wait_dbs()

        if self.robotConnector.RobotModel.SafetyStatus().StoppedDueToSafety:
            # self.robotConnector.DashboardClient.UnlockProtectiveStop()
            self.robotConnector.DashboardClient.ur_unlock_protective_stop()
            self.robotConnector.DashboardClient.wait_dbs()
            # self.robotConnector.DashboardClient.CloseSafetyPopup()
            self.robotConnector.DashboardClient.ur_close_safety_popup()
            self.robotConnector.DashboardClient.wait_dbs()
            # self.robotConnector.DashboardClient.BrakeRelease()
            self.robotConnector.DashboardClient.ur_brake_release()
            self.robotConnector.DashboardClient.wait_dbs()
            self.init_realtime_control()

        # return self.get_robot_status()['PowerOn'] & (not self.get_safety_status()['StoppedDueToSafety'])
        return self.robotConnector.RobotModel.RobotStatus().PowerOn and not self.robotConnector.RobotModel.SafetyStatus().StoppedDueToSafety

    def init_realtime_control(self):
        '''
        The realtime control mode enables continuous updates to a servoj program which is
        initialized by this function. This way no new program has to be sent to the robot
        and the robot can perform a smooth trajectory.
        Sending new servoj commands is done by utilizing RTDE of this library
        
        Parameters:
        sample_time: time of one sample, standard is 8ms as this is the thread-cycle time of UR
        
        Return Value:
        Status (bool): Status, True if successfully initialized.
        '''

        if not self.robotConnector.RTDE.isRunning():
            self.__logger.error('RTDE needs to be running to use realtime control')
            return False

        # get current tcp pos
        init_pose = self.get_actual_tcp_pose()

        self.robotConnector.RTDE.setData('input_double_register_0', init_pose[0])
        self.robotConnector.RTDE.setData('input_double_register_1', init_pose[1])
        self.robotConnector.RTDE.setData('input_double_register_2', init_pose[2])
        self.robotConnector.RTDE.setData('input_double_register_3', init_pose[3])
        self.robotConnector.RTDE.setData('input_double_register_4', init_pose[4])
        self.robotConnector.RTDE.setData('input_double_register_5', init_pose[5])

        self.robotConnector.RTDE.sendData()

        prog = '''def realtime_control():
    
    
    while (True):
        
        new_pose = p[read_input_float_register(0),
                    read_input_float_register(1),
                    read_input_float_register(2),
                    read_input_float_register(3),
                    read_input_float_register(4),
                    read_input_float_register(5)]
           
        servoj(get_inverse_kin(new_pose), t=0.2, lookahead_time= 0.1, gain=350)
            
        sync()
    end
end
'''
        # , t=0.1

        self.robotConnector.RealTimeClient.SendProgram(prog.format(**locals()))
        self.robotConnector.RobotModel.realtimeControlFlag = True

    def set_realtime_pose(self, pose):
        """
        Update/Set realtime_pose after sample_time seconds.

        Parameters
        pose: pose to transition to in sample_time seconds
        sample_time: time to take to perform servoj to next pose. 0.008 = thread cycle time of Universal Robot
        """

        if not self.robotConnector.RobotModel.realtimeControlFlag:
            print("Realtime control not initialized!")
            self.init_realtime_control()
            print("Realtime control initialized!")

        if self.robotConnector.RTDE.isRunning() and self.robotConnector.RobotModel.realtimeControlFlag:
            self.robotConnector.RTDE.setData('input_double_register_0', pose[0])
            self.robotConnector.RTDE.setData('input_double_register_1', pose[1])
            self.robotConnector.RTDE.setData('input_double_register_2', pose[2])
            self.robotConnector.RTDE.setData('input_double_register_3', pose[3])
            self.robotConnector.RTDE.setData('input_double_register_4', pose[4])
            self.robotConnector.RTDE.setData('input_double_register_5', pose[5])
            self.robotConnector.RTDE.sendData()
            return True
        else:
            if not self.robotConnector.RobotModel.realtimeControlFlag:
                self.__logger.warning('Realtime Remote Control not initialized')
            else:
                self.__logger.warning('RTDE is not running')

            return False
        
    def set_tools(self, STATE = "RELEASE"):
        
        if not self.robotConnector.RobotModel.realtimeControlFlag:
            print("Realtime control not initialized!")
            self.init_realtime_control()
            print("Realtime control initialized!")

        if self.robotConnector.RTDE.isRunning() and self.robotConnector.RobotModel.realtimeControlFlag:
            if STATE == "CLAMP":
                self.set_standard_digital_out(1, True)      
                self.set_standard_digital_out(2, False)  
            
            if STATE == "RELEASE":
                self.set_standard_digital_out(2, True)      
                self.set_standard_digital_out(1, False)     
                
            self.robotConnector.RTDE.sendData()
            
            return True
        else:
            if not self.robotConnector.RobotModel.realtimeControlFlag:
                self.__logger.warning('Realtime Remote Control not initialized')
            else:
                self.__logger.warning('RTDE is not running')

            return False
        
    def print_actual_tcp_pose(self):
        '''
        print the actual TCP pose
        '''
        self.print_pose(self.get_actual_tcp_pose())

    def print_actual_joint_positions(self):
        '''
        print the actual TCP pose
        '''
        self.print_pose(q=self.get_actual_joint_positions())

    def print_pose(self, pose=None, q=None):
        '''
        print a pose
        '''
        if q is None:
            print('Robot Pose: [{: 06.4f}, {: 06.4f}, {: 06.4f},   {: 06.4f}, {: 06.4f}, {: 06.4f}]'.format(*pose))
        else:
            print('Robot joint positions: [{: 06.4f}, {: 06.4f}, {: 06.4f},   {: 06.4f}, {: 06.4f}, {: 06.4f}]'.format(
                *q))
