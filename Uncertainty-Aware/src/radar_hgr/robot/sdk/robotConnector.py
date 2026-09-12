from .dashboard import DashBoard
from .dataLog import DataLog
from .dataLogging import DataLogging
from .realTimeClient import RealTimeClient
from .robotModel import RobotModel
from .rtde import RTDE
#%%
class RobotConnector(object):

    def __init__(self,
                 robotModel,
                 host,
                 hasForceTorque=False,
                 conf_filename=None):

        if(False):
            assert isinstance(robotModel, RobotModel) 
        self.RobotModel = robotModel
        self.RobotModel.ipAddress = host
        self.RobotModel.hasForceTorqueSensor = hasForceTorque
        self.RealTimeClient = RealTimeClient(robotModel)
        self.DataLog = DataLog(robotModel)
        self.RTDE = RTDE(robotModel, conf_filename=conf_filename)
        self.DashboardClient = DashBoard(robotModel)
        self.ForceTourqe = None
        logger = DataLogging()
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
