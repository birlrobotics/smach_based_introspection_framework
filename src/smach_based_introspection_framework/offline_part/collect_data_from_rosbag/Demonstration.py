from Skill import Skill
from Anomaly import Anomaly
from smach_based_introspection_framework.msg import GoalVector

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

    @property
    def original_goal(self):
        return getattr(self, '_original_goal', None)

    @original_goal.setter
    def original_goal(self, goal):
        if isinstance(goal, tuple):
            self._original_goal = goal
        else:
            raise Exception("Invalid original_goal, expected %s got %s"%(list, type(goal)))



    def __str__(self):
        sb = []
        for key in self.__dict__:
            sb.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))
     
        return "{%s:%s}"%(str(type(self)), ', '.join(sb))
 
    def __repr__(self):
        return self.__str__() 

