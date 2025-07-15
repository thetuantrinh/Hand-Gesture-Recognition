import ctypes
from src import UR
import numpy as np
import time
#%%
class UrScript(object):

    def __init__(self, 
                 host,
                 robotModel,
                 hasForceTorque=False,
                 conf_filename=None):
        '''
        Constructor see class description for more info.
        '''
        logger = UR.dataLogging.DataLogging()
        name = logger.AddEventLogging(__name__)
        self.__logger = logger.__dict__[name]
        self.robotConnector = UR.robotConnector.RobotConnector(
            robotModel,
            host,
            hasForceTorque, 
            conf_filename=conf_filename)
        #time.sleep(200)
        while(self.robotConnector.RobotModel.ActualTCPPose() is None):
            print("waiting for everything to be ready")
            time.sleep(1)
        self.__logger.info('Init done')
#%% #############   Module motion   ###############

    def waitRobotIdleOrStopFlag(self):

        while(self.robotConnector.RobotModel.RuntimeState() and not self.robotConnector.RobotModel.StopRunningFlag()):
            time.sleep(0.002)

        if self.robotConnector.RobotModel.rtcProgramExecutionError:

            print('Robot program execution error!!!')
            #raise RuntimeError('Robot program execution error!!!')

    def movej(self, q=None, a=1.4, v =1.05, t =0, r =0, wait=True, pose=None):
        '''
        Move to position (linear in joint-space) When using this command, the
        robot must be at standstill or come from a movej og movel with a
        blend. The speed and acceleration parameters controls the trapezoid
        speed profile of the move. The $t$ parameters can be used in stead to
        set the time for this move. Time setting has priority over speed and
        acceleration settings. The blend radius can be set with the $r$
        parameters, to avoid the robot stopping at the point. However, if he
        blend region of this mover overlaps with previous or following regions,
        this move will be skipped, and an 'Overlapping Blends' warning
        message will be generated.
        Parameters:
        q:    joint positions (Can also be a pose)
        a:    joint acceleration of leading axis [rad/s^2]
        v:    joint speed of leading axis [rad/s]
        t:    time [S]
        r:    blend radius [m]
        wait: function return when movement is finished
        pose: target pose
        '''
        prg =  '''def move_j():
{movestr}
end
'''
        movestr = self._move(movetype='j', pose=pose, a=a, v=v, t=t, r=r, wait=wait, q=q)

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def movel(self, pose=None, a=1.2, v =0.25, t =0, r =0, wait=True, q=None):
        '''
        Move to position (linear in tool-space)
        See movej.
        Parameters:
        pose: target pose (Can also be a joint position)
        a:    tool acceleration [m/s^2]
        v:    tool speed [m/s]
        t:    time [S]
        r:    blend radius [m]
        wait: function return when movement is finished
        q:    joint position
        '''

        prg =  '''def move_l():
{movestr}
end
'''
        movestr = self._move(movetype='l', pose=pose, a=a, v=v, t=t, r=r, wait=wait, q=q)

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        #time.sleep(0.5)
        if(wait):
            self.waitRobotIdleOrStopFlag()



    def movep(self, pose=None, a=1.2, v =0.25, r =0, wait=True, q=None):
        '''
        Move Process

        Blend circular (in tool-space) and move linear (in tool-space) to
        position. Accelerates to and moves with constant tool speed v.
        Parameters:
        pose: list of target pose (pose can also be specified as joint
              positions, then forward kinematics is used to calculate the corresponding pose)
        a:    tool acceleration [m/s^2]
        v:    tool speed [m/s]
        r:    blend radius [m]
        wait: function return when movement is finished
        q:    list of target joint positions
        '''

        prg =  '''def move_p():
{movestr}
end
'''
        movestr = self._move(movetype='p', pose=pose, a=a, v=v, t=0, r=r, wait=wait, q=q)
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()


    def movec(self, pose_via=None, pose_to=None, a=1.2, v =0.25, r =0, wait=True, q_via=None, q_to=None):
        '''
        Move Circular: Move to position (circular in tool-space)

        TCP moves on the circular arc segment from current pose, through pose via to pose to.
        Accelerates to and moves with constant tool speed v.

        Parameters:
        pose_via: path point (note: only position is used). (pose via can also be specified as joint positions,
                  then forward kinematics is used to calculate the corresponding pose)
        pose_to:  target pose (pose to can also be specified as joint positions, then forward kinematics
                  is used to calculate the corresponding pose)
        a:        tool acceleration [m/s^2]
        v:        tool speed [m/s]
        r:        blend radius (of target pose) [m]
        wait:     function return when movement is finished
        q_via:    list of via joint positions
        q_to:     list of target joint positions
        '''

        prg =  '''def move_p():
{movestr}
end
'''
        movestr = self._move(movetype='p', pose=pose_to, a=a, v=v, t=0, r=r, wait=wait, q=q_to,pose_via=pose_via, q_via=q_via)

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()


    def _move(self, movetype, pose=None, a=1.2, v=0.25, t=0, r=0, wait=True, q=None, pose_via=None, q_via=None):
        '''
        General move Process

        Blend circular (in tool-space) and move linear (in tool-space) to
        position. Accelerates to and moves with constant tool speed v.
        Parameters:
        movetype: j, l, p, c
        pose: list of target pose (pose can also be specified as joint
              positions, then forward kinematics is used to calculate the corresponding pose)
        a:    tool acceleration [m/s^2]
        v:    tool speed [m/s]
        r:    blend radius [m]
        wait: function return when movement is finished
        q:    list of target joint positions
        '''

        prefix="p"
        t_val=''
        pose_via_val=''
        if pose is None:
            prefix=""
            pose=q
        pose = np.array(pose)
        if movetype == 'j' or movetype == 'l':
            tval='t={t},'.format(**locals())

        if movetype =='c':
            if pose_via is None:
                prefix_via=""
                pose_via=q_via
            else:
                prefix_via="p"

            pose_via = np.array(pose_via)

            #Check if pose and pose_via have same shape
            if (pose.shape != pose_via.shape):
                return False

        movestr = ''
        if np.size(pose.shape)==2:
            for idx in range(np.size(pose, 0)):
                posex = np.round(pose[idx], 4)
                posex = posex.tolist()
                if movetype =='c':
                    pose_via_x = np.round(pose_via[idx], 4)
                    pose_via_x = pose_via_x.tolist()
                    pose_via_val='{prefix_via}{pose_via_x},'

                if (np.size(pose, 0)-1)==idx:
                    r=0
                movestr +=  '    move{movetype}({pose_via_val} {prefix}{posex}, a={a}, v={v}, {t_val} r={r})\n'.format(**locals())

            movestr +=  '    stopl({a})\n'.format(**locals())
        else:
            posex = np.round(pose, 4)
            posex = posex.tolist()
            if movetype =='c':
                pose_via_x = np.round(pose_via, 4)
                pose_via_x = pose_via_x.tolist()
                pose_via_val='{prefix_via}{pose_via_x},'
            movestr +=  '    move{movetype}({pose_via_val} {prefix}{posex}, a={a}, v={v}, {t_val} r={r})\n'.format(**locals())



        return movestr

    def force_mode(self, task_frame=[0.,0.,0., 0.,0.,0.], selection_vector=[0,0,1,0,0,0], wrench=[0.,0.,0., 0.,0.,0.], f_type=2, limits=[2, 2, 1.5, 1, 1, 1], wait=False, timeout=60):
        '''
        Set robot to be controlled in force mode

        Parameters:
        task frame: A pose vector that defines the force frame relative to the base frame.

        selection vector: A 6d vector that may only contain 0 or 1. 1 means that the robot will be
                          compliant in the corresponding axis of the task frame, 0 means the robot is
                          not compliant along/about that axis.

        wrench: The forces/torques the robot is to apply to its environment. These values
                have different meanings whether they correspond to a compliant axis or not.
                Compliant axis: The robot will adjust its position along/about the axis in order
                to achieve the specified force/torque. Non-compliant axis: The robot follows
                the trajectory of the program but will account for an external force/torque
                of the specified value.

        f_type: An integer specifying how the robot interprets the force frame.
                1: The force frame is transformed in a way such that its y-axis is aligned with a vector
                   pointing from the robot tcp towards the origin of the force frame.
                2: The force frame is not transformed.
                3: The force frame is transformed in a way such that its x-axis is the projection of
                   the robot tcp velocity vector onto the x-y plane of the force frame.
                All other values of f_type are invalid.

        limits: A 6d vector with float values that are interpreted differently for
                compliant/non-compliant axes:
                Compliant axes: The limit values for compliant axes are the maximum
                                allowed tcp speed along/about the axis.
                Non-compliant axes: The limit values for non-compliant axes are the
                                    maximum allowed deviation along/about an axis between the
                                    actual tcp position and the one set by the program.

        '''
        prg = '''def ur_force_mode():
        while True:
            force_mode(p{task_frame}, {selection_vector}, {wrench}, {f_type}, {limits})
            sync()
        end
end
'''

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def end_force_mode(self, wait=False):
        '''
        Resets the robot mode from force mode to normal operation.
        This is also done when a program stops.
        '''
        prg = 'end_force_mode()\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)                      ##### ToDo - check if send or sendprogram
        if(wait):
            self.waitRobotIdleOrStopFlag()
        time.sleep(0.05)

    def servoc(self, pose, a=1.2, v =0.25, r =0, wait=True):
        '''
        Servo Circular
        Servo to position (circular in tool-space). Accelerates to and moves with constant tool speed v.

        Parameters:
        pose: target pose
        a:    tool acceleration [m/s^2]
        v:    tool speed [m/s]
        r:    blend radius (of target pose) [m]
        '''
        prg = 'servoc(p{pose}, {a}, {v}, {r})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def servoj(self, q, t =0.008, lookahead_time=0.1, gain=100, wait=True):
        '''
        Servo to position (linear in joint-space)
        Servo function used for online control of the robot. The lookahead time
        and the gain can be used to smoothen or sharpen the trajectory.
        Note: A high gain or a short lookahead time may cause instability.
        Prefered use is to call this function with a new setpoint (q) in each time
        step (thus the default t=0.008)
        Parameters:
        q:              joint positions [rad]
        t:              time where the command is controlling
                        the robot. The function is blocking for time t [S]
        lookahead_time: time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
        gain:           proportional gain for following target position, range [100,2000]
        '''
        prg = 'servoj({q}, 0.5, 0.5, {t}, {lookahead_time}, {gain})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def speedj(self, qd, a, t , wait=True):
        '''
        Joint speed
        Accelerate linearly in joint space and continue with constant joint
        speed. The time t is optional; if provided the function will return after
        time t, regardless of the target speed has been reached. If the time t is
        not provided, the function will return when the target speed is reached.
        Parameters:
        qd: joint speeds [rad/s]
        a:  joint acceleration [rad/s^2] (of leading axis)
        t:  time [s] before the function returns (optional)
        '''
        prg = 'speedj({qd}, {a}, {t})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def stopj(self, a, wait=True):
        '''
        Stop (linear in joint space)
        Decellerate joint speeds to zero
        Parameters
        a: joint acceleration [rad/s^2] (of leading axis)
        '''
        prg = 'stopj({a})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def speedl(self, xd, a=1.4, t=0, aRot=None, wait=True):
        '''
        Tool speed
        Accelerate linearly in Cartesian space and continue with constant tool
        speed. The time t is optional; if provided the function will return after
        time t, regardless of the target speed has been reached. If the time t is
        not provided, the function will return when the target speed is reached.
        Parameters:
        xd:   tool speed [m/s] (spatial vector)
        a:    tool position acceleration [m/s^2]
        t:    time [s] before function returns (optional)
        aRot: tool acceleration [rad/s^2] (optional), if not defined a, position acceleration, is used
        '''
        if aRot is None:
            aRot=a
        prg = '''def ur_speedl():
    while(True):
        speedl({xd}, {a}, {t}, {aRot})
    end
end
'''
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
#         prg = 'speedl({xd}, {a}, {t}, {aRot})\n'
#         programString = prg.format(**locals())
#
#         self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def stopl(self, a=0.5, wait=True):
        '''
        Stop (linear in tool space)
        Decellerate tool speed to zero
        Parameters:
        a:    tool accleration [m/s^2]
        '''
        prg = 'stopl({a})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def freedrive_mode(self, wait=False):
        '''
        Set robot in freedrive mode. In this mode the robot can be moved around by hand in the
        same way as by pressing the "freedrive" button.
        The robot will not be able to follow a trajectory (eg. a movej) in this mode.
        '''
        prg = '''def ur_freedrive_mode():
    while(True):
        freedrive_mode()
        sleep(600)
    end
end
'''
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def end_freedrive_mode(self, wait=True):
        '''
        Set robot back in normal position control mode after freedrive mode.
        '''
        prg = 'end_freedrive_mode()\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()
        time.sleep(0.05)

    def teach_mode(self, wait=True):
        '''
        Set robot in freedrive mode. In this mode the robot can be moved
        around by hand in the same way as by pressing the "freedrive" button.
        The robot will not be able to follow a trajectory (eg. a movej) in this mode.
        '''
        prg = '''def ur_teach_mode():
    while True:
        teach_mode()
    end
end
'''
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def end_teach_mode(self, wait=True):
        '''
        Set robot back in normal position control mode after freedrive mode.
        '''
        prg = 'end_teach_mode()\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()
        time.sleep(0.05)

    def conveyor_pulse_decode(self, in_type, A, B, wait=True):
        '''
        Tells the robot controller to treat digital inputs number A and B as pulses
        for a conveyor encoder. Only digital input 0, 1, 2 or 3 can be used.

        >>> conveyor pulse decode(1,0,1)

        This example shows how to set up quadrature pulse decoding with
        input A = digital in[0] and input B = digital in[1]

        >>> conveyor pulse decode(2,3)

        This example shows how to set up rising and falling edge pulse
        decoding with input A = digital in[3]. Note that you do not have to set
        parameter B (as it is not used anyway).
        Parameters:
            in_type: An integer determining how to treat the inputs on A
                  and B
                  0 is no encoder, pulse decoding is disabled.
                  1 is quadrature encoder, input A and B must be
                    square waves with 90 degree offset. Direction of the
                    conveyor can be determined.
                  2 is rising and falling edge on single input (A).
                  3 is rising edge on single input (A).
                  4 is falling edge on single input (A).

            The controller can decode inputs at up to 40kHz
            A: Encoder input A, values of 0-3 are the digital inputs 0-3.
            B: Encoder input B, values of 0-3 are the digital inputs 0-3.
        '''

        prg = 'conveyor_pulse_decode({in_type}, {A}, {B})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def set_conveyor_tick_count(self, tick_count, absolute_encoder_resolution=0, wait=True):
        '''
        Tells the robot controller the tick count of the encoder. This function is
        useful for absolute encoders, use conveyor pulse decode() for setting
        up an incremental encoder. For circular conveyors, the value must be
        between 0 and the number of ticks per revolution.
        Parameters:
        tick_count: Tick count of the conveyor (Integer)
        absolute_encoder_resolution: Resolution of the encoder, needed to
                                     handle wrapping nicely.
                                     (Integer)
                                    0 is a 32 bit signed encoder, range [-2147483648 ;2147483647] (default)
                                    1 is a 8 bit unsigned encoder, range [0 ; 255]
                                    2 is a 16 bit unsigned encoder, range [0 ; 65535]
                                    3 is a 24 bit unsigned encoder, range [0 ; 16777215]
                                    4 is a 32 bit unsigned encoder, range [0 ; 4294967295]
        '''
        prg = 'set_conveyor_tick_count({tick_count}, {absolute_encoder_resolution})\n'
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def get_conveyor_tick_count(self):
        '''
        Tells the tick count of the encoder, note that the controller interpolates tick counts to get
        more accurate movements with low resolution encoders

        Return Value:
            The conveyor encoder tick count
        '''

        prg = '''def ur_get_conveyor_tick_count():
    write_output_float_register(0, get_conveyor_tick_count())
end
'''
        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.SendProgram(programString)
        self.waitRobotIdleOrStopFlag()
        return self.robotConnector.RobotModel.outputDoubleRegister[0]

    def stop_conveyor_tracking(self, a=15, aRot ='a', wait=True):
        '''
        Stop tracking the conveyor, started by track conveyor linear() or
        track conveyor circular(), and decellerate tool speed to zero.
        Parameters:
        a:    tool accleration [m/s^2] (optional)
        aRot: tool acceleration [rad/s^2] (optional), if not defined a, position acceleration, is used
        '''
        prg = 'stop_conveyor_tracking({a}, {aRot})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()


    def track_conveyor_circular(self, center, ticks_per_revolution, rotate_tool, wait=True):
        '''
        Makes robot movement (movej() etc.) track a circular conveyor.

        >>> track conveyor circular(p[0.5,0.5,0,0,0,0],500.0, false)

        The example code makes the robot track a circular conveyor with
        center in p[0.5,0.5,0,0,0,0] of the robot base coordinate system, where
        500 ticks on the encoder corresponds to one revolution of the circular
        conveyor around the center.
        Parameters:
        center:               Pose vector that determines the center the conveyor in the base
                              coordinate system of the robot.
        ticks_per_revolution: How many tichs the encoder sees when the conveyor moves one revolution.
        rotate tool:          Should the tool rotate with the coneyor or stay in the orientation
                              specified by the trajectory (movel() etc.).
        '''
        prg = 'track_conveyor_circular({center}, {ticks_per_revolution}, {rotate_tool})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()



    def track_conveyor_linear(self, direction, ticks_per_meter, wait=True):
        '''
        Makes robot movement (movej() etc.) track a linear conveyor.

        >>> track conveyor linear(p[1,0,0,0,0,0],1000.0)

        The example code makes the robot track a conveyor in the x-axis of
        the robot base coordinate system, where 1000 ticks on the encoder
        corresponds to 1m along the x-axis.
        Parameters:
        direction:       Pose vector that determines the direction of the conveyor in the base
                         coordinate system of the robot
        ticks per meter: How many tichs the encoder sees when the conveyor moves one meter
        '''
        prg = 'track_conveyor_linear({direction}, {ticks_per_meter})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def position_deviation_warning(self, enabled, threshold =0.8, wait=True):
        '''
        Write a message to the log when the robot position deviates from the target position.
        Parameters:
        enabled:   enable or disable position deviation log messages (Boolean)
        threshold: (optional) should be a ratio in the range ]0;1], where 0 is no position deviation and 1 is the
                   position deviation that causes a protective stop (Float).
        '''
        prg = 'position_deviation_warning({enabled}, {threshold})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def reset_revolution_counter(self, qNear=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=True):
        '''
        Reset the revolution counter, if no offset is specified. This is applied on
        joints which safety limits are set to "Unlimited" and are only applied
        when new safety settings are applied with limitted joint angles.

        >>> reset revolution counter()

        Parameters:
        qNear: Optional parameter, reset the revolution counter to one close to the given qNear joint vector.
               If not defined, the joint's actual number of revolutions are used.
        '''
        prg = 'reset_revolution_counter(qNear)\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def set_pos(self, q, wait=True):
        '''
        Set joint positions of simulated robot
        Parameters
        q: joint positions
        '''
        prg = 'set_pos({q})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

#%%####################   Module internals    ####################

    def force(self, wait=True):
        '''
        Returns the force exerted at the TCP

        Return the current externally exerted force at the TCP. The force is the
        norm of Fx, Fy, and Fz calculated using get tcp force().
        Return Value
        The force in Newtons (float)
        '''
        if(wait):
            self.sync()
        return self.robotConnector.RobotModel.TcpForceScalar()


    def get_actual_joint_positions(self, wait=True):
        '''
        Returns the actual angular positions of all joints

        The angular actual positions are expressed in radians and returned as a
        vector of length 6. Note that the output might differ from the output of
        get target joint positions(), especially durring acceleration and heavy
        loads.

        Return Value:
        The current actual joint angular position vector in rad : [Base,
        Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
        '''
        if(wait):
            self.sync()
        return self.robotConnector.RobotModel.ActualQ()
        c_pose = self.robotConnector.RobotModel.ActualQ

        pose = []
        pose.append(ctypes.c_double(c_pose[0]).value)
        pose.append(ctypes.c_double(c_pose[1]).value)
        pose.append(ctypes.c_double(c_pose[2]).value)
        pose.append(ctypes.c_double(c_pose[3]).value)
        pose.append(ctypes.c_double(c_pose[4]).value)
        pose.append(ctypes.c_double(c_pose[5]).value)
        
        return pose



    def get_actual_joint_speeds(self, wait=True):
        '''
        Returns the actual angular velocities of all joints

        The angular actual velocities are expressed in radians pr. second and
        returned as a vector of length 6. Note that the output might differ from
        the output of get target joint speeds(), especially durring acceleration
        and heavy loads.

        Return Value
        The current actual joint angular velocity vector in rad/s:
        [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
        '''
        if(wait):
            self.sync()
        return self.robotConnector.RobotModel.ActualQD


    def get_actual_tcp_pose(self, wait=True):
        '''
        Returns the current measured tool pose

        Returns the 6d pose representing the tool position and orientation
        specified in the base frame. The calculation of this pose is based on
        the actual robot encoder readings.

        Return Value
        The current actual TCP vector : ([X, Y, Z, Rx, Ry, Rz])
        '''
        if(wait):
            self.sync()
        return self.robotConnector.RobotModel.ActualTCPPose()
        c_pose = self.robotConnector.RobotModel.ActualTCPPose

        pose = []
        pose.append(ctypes.c_double(c_pose[0]).value)
        pose.append(ctypes.c_double(c_pose[1]).value)
        pose.append(ctypes.c_double(c_pose[2]).value)
        pose.append(ctypes.c_double(c_pose[3]).value)
        pose.append(ctypes.c_double(c_pose[4]).value)
        pose.append(ctypes.c_double(c_pose[5]).value)
        
        return pose

    def get_actual_tcp_speed(self,wait=True):
        '''
        Returns the current measured TCP speed

        The speed of the TCP retuned in a pose structure. The first three values
        are the cartesian speeds along x,y,z, and the last three define the
        current rotation axis, rx,ry,rz, and the length |rz,ry,rz| defines the angular
        velocity in radians/s.
        Return Value
        The current actual TCP velocity vector; ([X, Y, Z, Rx, Ry, Rz])
        '''
        if(wait):
            self.sync()

        return self.robotConnector.RobotModel.ActualTCPSpeed()

    def get_tcp_force(self, wait=True):
        '''
        Returns the wrench (Force/Torque vector) at the TCP

        The external wrench is computed based on the error between the joint
        torques required to stay on the trajectory and the expected joint
        torques. The function returns "p[Fx (N), Fy(N), Fz(N), TRx (Nm), TRy (Nm),
        TRz (Nm)]". where Fx, Fy, and Fz are the forces in the axes of the robot
        base coordinate system measured in Newtons, and TRx, TRy, and TRz
        are the torques around these axes measured in Newton times Meters.

        Return Value:
        the wrench (pose)
        '''
        if(wait):
            self.sync()
        return self.robotConnector.RobotModel.ActualTCPForce()

    def set_payload_mass(self, m, wait=True):
        '''
        Set payload mass

        See also set payload.

        Sets the mass of the payload.

        This function must be called, when the payload weight changes - i.e
        when the robot picks up or puts down a heavy workpiece.

        Parameters:
        m: mass in kilograms
        '''
        prg = 'set_payload_mass({m})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()

    def set_tcp(self, pose, wait=True):
        '''
        Set the Tool Center Point

        Sets the transformation from the output flange coordinate system to
        the TCP as a pose.

        Parameters:
        pose: A pose describing the transformation.
        '''

        if type(pose).__module__ == np.__name__:
            pose = pose.tolist()
        prg = 'set_tcp(p{pose})\n'

        programString = prg.format(**locals())

        self.robotConnector.RealTimeClient.Send(programString)
        if(wait):
            self.waitRobotIdleOrStopFlag()
        time.sleep(0.05)
        
    def set_standard_digital_out(self, n, b):
        '''
        Set standard digital output signal level

        See also set configurable digital out and set tool digital out.

        Parameters:
        n: The number (id) of the input, integer: [0:7]
        b: The signal level. (boolean)
        '''
        if b:
            self.robotConnector.RTDE.setData('standard_digital_output_mask', 2**n)
            self.robotConnector.RTDE.setData('standard_digital_output', 2**n)
        else:
            self.robotConnector.RTDE.setData('standard_digital_output_mask', 2**n)
            self.robotConnector.RTDE.setData('standard_digital_output', 0)
        self.robotConnector.RTDE.sendData()
        self.robotConnector.RTDE.setData('standard_digital_output_mask', 0)
        self.robotConnector.RTDE.setData('standard_digital_output', 0) 

    def sleep(self, t):
        '''
        Sleep for an amount of time

        Parameters:
        t: time [s]
        '''
        time.sleep(t)

    def sync(self):
        '''
        Uses up the remaining "physical" time a thread has in the current
        frame/sample.
        '''
        initialRobotTime = self.robotConnector.RobotModel.RobotTimestamp()
        while(self.robotConnector.RobotModel.RobotTimestamp() == initialRobotTime):
            time.sleep(0.001)
