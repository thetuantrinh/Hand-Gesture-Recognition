from src import UR
import threading
import socket
import struct
import select
import time

DEFAULT_TIMEOUT = 2.0

class ConnectionState:
    ERROR = 0
    DISCONNECTED = 1
    CONNECTED = 2
    PAUSED = 3
    STARTED = 4


class DashBoard(threading.Thread): 
    def __init__(self, robotModel):
        '''
        Constructor see class description for more info.
        '''
        if(False):
            assert isinstance(robotModel, UR.robotModel.RobotModel) 
        self.__robotModel = robotModel

        logger = UR.dataLogging.DataLogging()
        name = logger.AddEventLogging(__name__)        
        self._logger = logger.__dict__[name]
        self.__reconnectTimeout = 60 #Seconds (while in run)
        self.__conn_state = ConnectionState.DISCONNECTED
        self.last_respond = None
        self.__stop_event = True
        threading.Thread.__init__(self)
        self.__dataEvent = threading.Condition()
        self.__dataAccess = threading.Lock()
        self.__sock = None
        self.start()
        self.wait_dbs()
        self._logger.info('Dashboard server constructor done')


    def ur_load(self, file):
        '''
        Load the specified program. Return when loading has completed.
        
        Return value to Log file:
        "Loading program: <program.urp>" OR "File not found: <program.urp>"
        '''
        self.__send('load ' + file + '\n')

    def ur_play(self):
        '''
        Starts program, if any program is loaded and robot is ready. Return when the program execution has been started.

        Return value to Log file:
        "Starting program"
        '''
        self.__send('play\n')
        
    def ur_stop(self):
        '''
        Stops running program and returns when stopping is completed.
        
        Return value to Log file:
        "Stopped"
        '''
        self.__send('stop\n')


    def ur_pause(self):
        '''
        Pauses the running program and returns when pausing is completed.
        
        Return value to Log file:
        "Pausing program"
        '''
        self.__send('pause\n')


    def ur_shutdown(self):
        '''
        Shuts down and turns off robot and controller.
        
        Return value to Log file:
        "Shutting down"
        '''
        self.__send('shutdown\n')
        
    def ur_running(self):
        '''
        Execution state enquiry.
        
        Return value to Log file:
        "Robot running: True" OR "Robot running: False"
        '''
        self.__send('running\n')
        
    def ur_robotmode(self):
        '''
        Robot mode enquiry
        
        Return value to Log file:
        "Robotmode: <mode>", where <mode> is:        
        NO_CONTROLLER
        DISCONNECTED
        CONFIRM_SAFETY
        BOOTING
        POWER_OFF
        POWER_ON
        IDLE
        BACKDRIVE
        RUNNING
        '''
        self.__send('robotmode\n')

    def ur_get_loaded_program(self):
        '''
        Which program is loaded.
        
        Return value to Log file:
        "Program loaded: <path to loaded program file>" OR "No program loaded"
        '''
        self.__send('get loaded program\n')

    def ur_popup(self,  popupText=''):
        '''
        The popup-text will be translated to the selected language, if the text exists in the language file.
        
        Return value to Log file:
        "showing popup"
        '''
        self.__send('popup ' + popupText + '\n')

    def ur_close_popup(self):
        '''
        Closes the popup.
        
        Return value to Log file:
        "closing popup"
        '''
        self.__send('close popup\n')

    def ur_addToLog(self, logMessage):
        '''
        Adds log-message to the Log history.

        Return value to Log file:
        "Added log message" Or "No log message to add"
        '''
        self.__send('addToLog ' + logMessage + '\n')

    def ur_setUserRole(self, role):
        '''
        Simple control of user privileges: controls the available options on the Welcome screen.
        
        Return value to Log file:
        "Setting user role: <role>" OR "Failed setting user role: <role>"
        '''
        self.__send('setUserRole ' + role + '\n')

    def ur_isProgramSaved(self):
        '''
        Returns the save state of the active program.
        
        Return value to Log file:
        "True" OR "False"
        '''
        self.__send('isProgramSaved\n')

    def ur_programState(self):
        '''
        Returns the state of the active program, or STOPPED if no program is loaded.
        
        Return value to Log file:
        "STOPPED" if no program is running OR "PLAYING" if program is running
        '''
        self.__send('programState\n')

    def ur_polyscopeVersion(self):
        '''
        Returns the version of the Polyscope software.
        
        Return value to Log file:
        version number, like "3.0.15547"
        '''
        self.__send('polyscopeVersion\n')

    def ur_setUserRole_where(self, role, level):
        '''
        "setUserRole <role>, where <role> is"
        programmer = "SETUP Robot" button is disabled, "Expert Mode" is available (if correct password is supplied)
        operator = Only "RUN Program" and "SHUTDOWN Robot" buttons are enabled, "Expert Mode" cannot be activated
        none ( or send setUserRole) = All buttons enabled, "Expert Mode" is available (if correct password is supplied)
        locked = All buttons disabled and "Expert Mode" cannot be activated
        Control of user privileges: controls the available options on the Welcome screen.
        
        Note: If the Welcome screen is not active when the command is sent, 
        the user privileges defined by the new user role will not be effective 
        until the user switches to the Welcome screen.

        Return value to Log file:
        "Setting user role: <role>" OR "Failed setting user role: <role>"
        '''
        self.__send('setUserRole '+ role + ', where ' + role + ' is' + level +'\n')

    def ur_power_on(self):
        '''
        Powers on the robot arm.
        
        Return value to Log file:
        "Powering on"
        '''
        self.__send('power on\n')

    def ur_power_off(self):
        '''
        Powers off the robot arm.
        
        Return value to Log file:
        "Powering off"
        '''
        self.__send('power off\n')

    def ur_brake_release(self):
        '''
        Releases the brakes.
        
        Return value to Log file:
        "Brake releasing"        
        '''
        self.__send('brake release\n')

    def ur_safetymode(self):
        '''
        Safety mode enquiry.
        
        Return value to Log file:
        "safety mode: <mode>", where <mode> is
        
        NORMAL
        REDUCED
        PROTECTIVE_STOP
        RECOVERY
        SAFEGUARD_STOP
        SYSTEM_EMERGENCY_STOP
        ROBOT_EMERGENCY_STOP
        VIOLATION
        FAULT        
        '''
        return self.__send('safetymode\n')

    def ur_unlock_protective_stop(self):
        '''
        Closes the current popup and unlocks protective stop.
        
        Return value to Log file:
        "Protective stop releasing"
        '''
        self.__send('unlock protective stop\n')

    def ur_close_safety_popup(self):
        '''
        Closes a safety popup.
        
        Return value to Log file:
        "closing safety popup"        
        '''
        self.__send('close safety popup\n')

    def ur_load_installation(self, instal='default.installation'):
        '''
        Loads the specified installation file.
        
        Return value to Log file:
        "Loading installation: <default.installation>" OR "File not found: <default.installation>"
        '''
        self.__send('load installation '+ instal +'\n')

    def __connect(self):
        '''
        Initialize DashBoard connection to host.
        
        Return value:
        success (boolean)
        '''       
        if self.__sock:
            return True

        t0 = time.time()
        while (time.time()-t0<self.__reconnectTimeout) and self.__conn_state < ConnectionState.CONNECTED:
            try:
                self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)            
                self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)         
                self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.__sock.settimeout(DEFAULT_TIMEOUT)
                self.__sock.connect((self.__robotModel.ipAddress, 29999))
                self.__conn_state = ConnectionState.CONNECTED
                time.sleep(0.5)
                self._logger.info('Connected')
                return True
            except (socket.timeout, socket.error):
                self.__sock = None
                self._logger.error('Dashboard connecting')

        return False

    def close(self):
        '''
        Close the DashBoard connection.
        Example:
        rob = URBasic.dashboard.DashBoard('192.168.56.101', rtde_conf_filename='rtde_configuration.xml', logger=logger)
        rob.close_dbs()
        '''
#        if self.IsRtcConnected():
#            self.close_rtc()

        if self.__stop_event is False:
            self.__stop_event = True
            self.join()
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.__conn_state = ConnectionState.DISCONNECTED
        return True

    def dbs_is_running(self):
        '''
        Return True if Dash Board server is running
        '''
        return self.__conn_state >= ConnectionState.STARTED

    def run(self):
        self.__stop_event = False
        t0 = time.time()
        while (time.time()-t0<self.__reconnectTimeout) and self.__conn_state < ConnectionState.CONNECTED:
            if not self.__connect():
                self._logger.warning("UR Dashboard connection failed!")

        if self.__conn_state < ConnectionState.CONNECTED:
            self._logger.error("UR Dashboard interface not able to connect and timed out!")
            return
        
        while (not self.__stop_event) and (time.time()-t0<self.__reconnectTimeout):
            try:
                msg = self.__receive()
                if msg is not None:
                    self._logger.info('UR Dashboard respond ' + msg)
                    self.last_respond = msg

                with self.__dataEvent:
                    self.__dataEvent.notifyAll()
                t0 = time.time()
                self.__conn_state = ConnectionState.STARTED

            except Exception:
                if self.__conn_state >= ConnectionState.CONNECTED:
                    self.__conn_state = ConnectionState.ERROR
                    self._logger.error("Dashboard server interface stopped running")

                    try:
                        self.__sock.close()
                    except:
                        pass
                    self.__sock = None
                    self.__connect()

                if self.__conn_state >= ConnectionState.CONNECTED:
                    self._logger.info("Dashboard server interface reconnected")
                else:
                    self._logger.warning("Dashboard server reconnection failed!")

        self.__conn_state = ConnectionState.PAUSED
        with self.__dataEvent:
            self.__dataEvent.notifyAll()
        self._logger.info("Dashboard server interface is stopped")

    def wait_dbs(self):
        '''Wait while the data receiving thread is receiving a new message.'''
        with self.__dataEvent:
            self.__dataEvent.wait()
        
    def __send(self, cmd):
        '''
        Send command to Robot Controller. 

        Input parameters:
        cmd (str)

        Return value:
        success (boolean)
        '''
        t0 = time.time()
        while (time.time()-t0<self.__reconnectTimeout):
            try:
                buf = bytes(cmd, 'utf-8')
                (_, writable, _) = select.select([], [self.__sock], [], DEFAULT_TIMEOUT)
                if len(writable):
                    self.__sock.sendall(buf)
                    self.wait_dbs()
                    return True
            except:
                self._logger.error('Could not send program!')

        self._logger.error('Program re-sending timed out - Could not send program!')
        return False

    def __receive(self):
        '''
        Receive the respond a send command from the Robot Controller. 

        Return value:
        Output from Robot controller (type is depended on the input parameters)
        '''
        (readable, _, _) = select.select([self.__sock], [], [], DEFAULT_TIMEOUT)
        if len(readable):
            data = self.__sock.recv(1024)
            if len(data) == 0:
                return None
            
            fmt = ">" + str(len(data)) + "B"
            out =  struct.unpack_from(fmt, data)        
            return ''.join(map(chr,out[:-1]))