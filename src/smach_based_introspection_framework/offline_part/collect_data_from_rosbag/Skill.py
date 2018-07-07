import genpy

class Skill(object):
    def __init__(self):
        pass

    @property
    def tag(self):
        return getattr(self, '_tag', None)

    @tag.setter
    def tag(self, tag):
        if isinstance(tag, str):
            self._tag = tag
        else:
            raise Exception("Invalid tag, expected %s got %s"%(str, type(tag)))

    @property
    def start_time(self):
        return getattr(self, '_start_time', None)

    @start_time.setter
    def start_time(self, start_time):
        if isinstance(start_time, genpy.Time):
            self._start_time = start_time
        else:
            raise Exception("Invalid start_time, expected %s got %s"%(genpy.Time, type(start_time)))

    @property
    def end_time(self):
        return getattr(self, '_end_time', None)

    @end_time.setter
    def end_time(self, end_time):
        if isinstance(end_time, genpy.Time):
            self._end_time = end_time
        else:
            raise Exception("Invalid end_time, expected %s got %s"%(genpy.Time, type(end_time)))

    def __str__(self):
        sb = []
        for key in self.__dict__:
            sb.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))
     
        return "{%s:%s}"%(str(type(self)), ', '.join(sb))
 
    def __repr__(self):
        return self.__str__() 
