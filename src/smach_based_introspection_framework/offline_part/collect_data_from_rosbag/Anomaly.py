import genpy
from Skill import Skill

class Anomaly(object):
    def __init__(self):
        pass

    @property
    def label(self):
        return getattr(self, '_label', None)

    @label.setter
    def label(self, label):
        if isinstance(label, str):
            self._label = label
        else:
            raise Exception("Invalid label, expected %s got %s"%(str, type(label)))

    @property
    def time(self):
        return getattr(self, '_time', None)

    @time.setter
    def time(self, time):
        if isinstance(time, genpy.Time):
            self._time = time
        else:
            raise Exception("Invalid time, expected %s got %s"%(genpy.Time, type(time)))

    @property
    def skill_belonged_to(self):
        return getattr(self, '_skill', None)

    @skill_belonged_to.setter
    def skill_belonged_to(self, skill):
        if isinstance(skill, Skill):
            self._skill = skill
        else:
            raise Exception("Invalid skill, expected %s got %s"%(Skill, type(skill)))

    def __str__(self):
        sb = []
        for key in self.__dict__:
            sb.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))
     
        return "{%s:%s}"%(str(type(self)), ', '.join(sb))
 
    def __repr__(self):
        return self.__str__() 
