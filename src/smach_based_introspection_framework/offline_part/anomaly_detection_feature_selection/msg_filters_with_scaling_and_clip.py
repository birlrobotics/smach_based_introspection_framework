from rostopics_to_timeseries import TopicMsgFilter
import numpy as np

class TactileStaticStdScaleClipMaxFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdScaleClipMaxFilter, self).__init__()

    def convert(self, msg):
        ret = np.array([
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ])
        return [np.clip(ret/60.0, -1, 1).max()]
        

    @staticmethod
    def vector_size():
        return 1

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std.clip(ret/60.0, -1, 1).max()', \
        ] 


class TactileStaticStdScaleClipFilter(TopicMsgFilter):
    def __init__(self):
        super(TactileStaticStdScaleClipFilter, self).__init__()

    def convert(self, msg):
        ret = np.array([
            np.std(msg.taxels[0].values),
            np.std(msg.taxels[1].values),
        ])
        return np.clip(ret/60.0, -1, 1)
        

    @staticmethod
    def vector_size():
        return 2

    @staticmethod
    def vector_meaning():
        return [
            'tactile_static_data.left.std.clip(ret/60.0, -1, 1)', \
            'tactile_static_data.right.std.clip(ret/60.0, -1, 1)', \
        ] 

        

