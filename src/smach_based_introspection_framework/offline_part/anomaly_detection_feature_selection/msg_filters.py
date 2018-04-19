from rostopics_to_timeseries import TopicMsgFilter

class WrenchStampedFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ]

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']] 
