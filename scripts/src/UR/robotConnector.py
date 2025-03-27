from src import UR
#%%
class RobotConnector(object):

    def __init__(self,
                 robotModel,
                 host,
                 hasForceTorque=False,
                 conf_filename=None):

        if(False):
            assert isinstance(robotModel, UR.robotModel.RobotModel) 
        self.RobotModel = robotModel
        self.RobotModel.ipAddress = host
        self.RobotModel.hasForceTorqueSensor = hasForceTorque
        self.RealTimeClient = UR.realTimeClient.RealTimeClient(robotModel)
        self.DataLog = UR.dataLog.DataLog(robotModel)
        self.RTDE = UR.rtde.RTDE(robotModel, conf_filename=conf_filename)
        self.DashboardClient = UR.dashboard.DashBoard(robotModel)
        self.ForceTourqe = None
        logger = UR.dataLogging.DataLogging()
        name = logger.AddEventLogging(__name__)
        self.__logger = logger.__dict__[name]
        self.__logger.info('Init done')
#%%
    def close(self):
        self.DataLog.close()
        self.RTDE.close()
        self.RealTimeClient.Disconnect()
        self.DashboardClient.close()
        if self.ForceTourqe is not None:
            self.ForceTourqe.close()
