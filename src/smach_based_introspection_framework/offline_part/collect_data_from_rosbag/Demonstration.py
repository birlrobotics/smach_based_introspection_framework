from Skill import Skill
from Anomaly import Anomaly

class Demonstration(Skill):
    def __init__(self):
        Skill.__init__(self)

    @property
    def targeted_anomaly(self):
        return getattr(self, '_targeted_anomaly', None)

    @targeted_anomaly.setter
    def targeted_anomaly(self, anomaly):
        if isinstance(anomaly, Anomaly):
            self._targeted_anomaly = anomaly
        else:
            raise Exception("Invalid targeted_anomaly, expected %s got %s"%(Anomaly, type(targeted_anomaly)))

    def __str__(self):
        sb = []
        for key in self.__dict__:
            sb.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))
     
        return "{%s:%s}"%(str(type(self)), ', '.join(sb))
 
    def __repr__(self):
        return self.__str__() 
